/**
 * @file submit.c
 * @brief Request submission, per-kind validation and staging, protocol API wrappers (write, read, spoll, allspoll, findrqs, reset, TCT, SEND COMMAND, PPOLL configure/unconfigure, FINDLSTN, REQUESTCTL).
 */
#include <stddef.h>
#include <string.h>

#include "gpib_internal.h"

/* Whole-list validation before any reservation: write_many with at least one
 * unsupported target is rejected atomically, before any listener is addressed
 * (ADDR-05); single-address operations reject before side effects (ADDR-04). */
static gpib_status_t validate_request_addresses(const gpib_request_t *req) {
    if (req->kind == GPIB_REQ_WRITE_MANY) {
        if (req->listeners == NULL || req->listener_count == 0) {
            return GPIB_INVALID_ARGUMENT;
        }
        if (req->listener_count > GPIB_CFG_MAX_LISTENERS) {
            /* Never silently truncate the listener list: the operation is
             * rejected before any bus side effect. */
            return GPIB_INVALID_ARGUMENT;
        }
        for (size_t i = 0; i < req->listener_count; i++) {
            if (!gpib_address_supported(&req->listeners[i])) {
                return GPIB_UNSUPPORTED_ADDRESSING;
            }
        }
        return GPIB_OK;
    }
    if (!gpib_address_supported(&req->address)) {
        return GPIB_UNSUPPORTED_ADDRESSING;
    }
    return GPIB_OK;
}

/* Device-role operations share the submit machinery with a runtime kind
 * check; the device data path itself lives in the line sequencer. */
static gpib_status_t device_exchange(gpib_context_t *ctx,
    gpib_request_kind_t kind, const void *data, size_t len,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    /* The controller never exchanges as an addressed device through this
     * API: it initiates via submit/serial_poll (REQ-017). */
    (void)kind;
    (void)data;
    (void)len;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#else
    {
        gpib_request_t r;
        memset(&r, 0, sizeof(r));
        r.kind = kind;
        if (kind == GPIB_REQ_DEV_TX) {
            r.tx_data = data;
            r.tx_len = len;
        } else {
            r.rx_data = (void *)data; /* borrowed sink buffer */
            r.rx_cap = len;
        }
        return gpib_submit(ctx, &r, txn_out, block_out);
    }
#endif
}

gpib_status_t gpib_device_supply(gpib_context_t *ctx, const void *data,
    size_t len, gpib_handle_t *txn_out, gpib_handle_t *block_out) {
    return device_exchange(ctx, GPIB_REQ_DEV_TX, data, len, txn_out,
        block_out);
}

gpib_status_t gpib_device_sink(gpib_context_t *ctx, void *buf, size_t cap,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
    return device_exchange(ctx, GPIB_REQ_DEV_RX, buf, cap, txn_out,
        block_out);
}

gpib_status_t gpib_pass_control(gpib_context_t *ctx, uint8_t address,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_TCT;
    r.address.primary = address;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)address;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_send_commands(gpib_context_t *ctx, const uint8_t *cmds,
    size_t n, gpib_handle_t *txn_out, gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (cmds == NULL || n == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (n > GPIB_CFG_MAX_CMDS) {
        /* Published capacity: reject before side effects, never silently
         * truncate the ATN byte list. */
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_CMDS;
    r.cmd_list = cmds;
    r.cmd_len = n;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)cmds;
    (void)n;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_submit(gpib_context_t *ctx, const gpib_request_t *req,
    gpib_handle_t *txn_out, gpib_handle_t *first_block_out) {
    if (ctx == NULL || req == NULL || txn_out == NULL ||
        first_block_out == NULL) {
        /* A13: EVERY accepted request creates a first block record whose
         * ownership returns only through its ACK -- a NULL output
         * pointer used to leak that record permanently (no public API
         * recovers the handle; deinit wedged in BUSY_RESOURCES). */
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE; /* default-deny for STOPPING/DRAINING */
    }
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    if (gpib_line_mode(ctx) && ctx->line.ctrl.main == GPIB_C_CIDS &&
        ctx->line.cic_once &&
        req->kind != GPIB_REQ_DEV_TX && req->kind != GPIB_REQ_DEV_RX) {
        /* Multi-controller arbitration: the instance HELD CIC and gave it
         * away (TCT) --- re-submission is refused until it explicitly
         * re-seizes via gpib_interface_clear (the SC's legitimate path).
         * A fresh SC at power-on (cic_once == false, IFC pulse in flight)
         * submits fine; dispatch is gated on the FSM reaching CIC.
         * A non-SC that never held CIC is refused outright. */
        return GPIB_NOT_CIC;
    }
    if (gpib_line_mode(ctx) && ctx->line.ctrl.main == GPIB_C_CIDS &&
        !ctx->system_controller &&
        req->kind != GPIB_REQ_DEV_TX && req->kind != GPIB_REQ_DEV_RX) {
        return GPIB_NOT_CIC;
    }
#endif
#if GPIB_CFG_ROLE == GPIB_ROLE_DEVICE
    /* A Device does not address other devices (REQ-017); its externally
     * initiated data path is gpib_device_supply/gpib_device_sink. */
    if (req->kind != GPIB_REQ_DEV_TX && req->kind != GPIB_REQ_DEV_RX) {
        return GPIB_UNSUPPORTED;
    }
#else
    /* Controller build: device-role exchanges are not addressable ops. */
    if (req->kind == GPIB_REQ_DEV_TX || req->kind == GPIB_REQ_DEV_RX) {
        return GPIB_UNSUPPORTED;
    }
#endif
    /* Payload contract per kind. The switch is deliberately DEFAULT-FREE
     * and exhaustive over the enum: -Werror=-Wswitch forces a new kind to
     * be reviewed both HERE and in the staging switch below, instead of
     * silently falling into a reject branch (the PPOLL/FINDLSTN
     * INVALID_ARGUMENT regression class). Out-of-enum garbage values are
     * caught by the bounds gate. */
    if ((unsigned)req->kind > (unsigned)GPIB_REQ_FINDLSTN) {
        return GPIB_INVALID_ARGUMENT;
    }
    switch (req->kind) {
    case GPIB_REQ_WRITE:
    case GPIB_REQ_WRITE_MANY:
    case GPIB_REQ_DEV_TX:
        /* H4 (final audit): a zero-length block would park the SH in
         * SGNS forever (no byte to source) -- the API rejects it; the
         * zero-length POLICY itself stays OPEN-23. */
        if (req->tx_data == NULL || req->tx_len == 0) {
            return GPIB_INVALID_ARGUMENT;
        }
        break;
    case GPIB_REQ_READ:
        if (req->rx_data == NULL || req->rx_cap == 0) {
            return GPIB_INVALID_ARGUMENT;
        }
        /* rx_min_bytes is a STREAM-level target and may exceed the first
         * block: further blocks arrive via submit_block and the gap is a
         * controlled backpressure pause (REQ-026/REQ-035). */
        break;
    case GPIB_REQ_SPOLL:
    case GPIB_REQ_TCT:
    case GPIB_REQ_PPOLL:
        break; /* staging is internal: no external payload */
    case GPIB_REQ_CMDS:
        /* F07: the low level owns this contract, not the wrapper -- a
         * NULL list, an empty list or an over-capacity list would either
         * dereference NULL or make the sequencer read past cmd_stored[]
         * (cmd_len bytes are read back at dispatch). Never truncate. */
        if (req->cmd_list == NULL || req->cmd_len == 0 ||
            req->cmd_len > GPIB_CFG_MAX_CMDS) {
            return GPIB_INVALID_ARGUMENT;
        }
        break;
    case GPIB_REQ_DEV_RX:
        /* F08: the sink buffer is written by the core; the contract is
         * enforced here, not only in the wrapper. */
        if (req->rx_data == NULL || req->rx_cap == 0) {
            return GPIB_INVALID_ARGUMENT;
        }
        break;
    case GPIB_REQ_ALLSPOLL:
    case GPIB_REQ_FINDRQS:
    case GPIB_REQ_RESET:
    case GPIB_REQ_FINDLSTN:
        /* F08/F12: the tx list is a primary-address list; 31 is the
         * reserved UNL/UNT encoding, larger values alias (5-bit field).
         * Results sinks are written by the core at completion; FINDRQS
         * also resolves *find_out at accounting time. */
        if (req->tx_data == NULL || req->tx_len == 0) {
            return GPIB_INVALID_ARGUMENT;
        }
        {
            const uint8_t *list = (const uint8_t *)req->tx_data;
            for (size_t i = 0; i < req->tx_len; i++) {
                if (list[i] > 30u) {
                    return GPIB_INVALID_ARGUMENT;
                }
            }
        }
        if ((req->kind == GPIB_REQ_ALLSPOLL ||
             req->kind == GPIB_REQ_FINDRQS ||
             req->kind == GPIB_REQ_FINDLSTN) && req->rx_data == NULL) {
            return GPIB_INVALID_ARGUMENT;
        }
        /* AUD-02: the results sink must hold EVERY entry the core will
         * write -- the ASan probe overflowed a 1-byte bool array with a
         * 2-address FINDLSTN. rx_cap is BYTES: FINDLSTN writes one bool
         * per address, ALLSPOLL/FINDRQS one STB byte per address. */
        if (req->kind == GPIB_REQ_ALLSPOLL || req->kind == GPIB_REQ_FINDRQS) {
            if (req->rx_cap < req->tx_len) {
                return GPIB_INVALID_ARGUMENT;
            }
        } else if (req->kind == GPIB_REQ_FINDLSTN) {
            if (req->rx_cap < req->tx_len * sizeof(bool)) {
                return GPIB_INVALID_ARGUMENT;
            }
        }
        if (req->kind == GPIB_REQ_FINDRQS && req->find_out == NULL) {
            return GPIB_INVALID_ARGUMENT;
        }
        break;
    }
    gpib_status_t st = validate_request_addresses(req);
    if (st != GPIB_OK) {
        return st; /* nothing captured, ledger untouched */
    }
    /* Atomic reservation: transaction terminal record + the first block slot
     * together or not at all (TXN-01). */
    int t = gpib_ledger_find_free_txn(ctx);
    int b = gpib_ledger_find_free_block(ctx);
    if (t < 0 || b < 0) {
        return GPIB_BUSY_RESOURCES; /* rejection transfers no ownership */
    }
    struct gpib_txn_record *tr = &ctx->txn[t];
    struct gpib_block_record *br = &ctx->block[b];
    tr->state = TXN_OPEN;
    tr->address = (req->kind == GPIB_REQ_WRITE_MANY) ? req->listeners[0]
                                                     : req->address;
    tr->listeners_first = (req->kind == GPIB_REQ_WRITE_MANY) ? req->listeners
                                                             : NULL;
    tr->listener_count = (req->kind == GPIB_REQ_WRITE_MANY)
                             ? req->listener_count
                             : 0;
    tr->kind = req->kind;
    tr->block_count = 1;
    tr->blocks_terminal = 0;
    tr->rx_limit = req->rx_min_bytes;
    tr->stream_bytes = 0; /* slot reuse must not leak the previous stream */
    tr->deadline_us = req->deadline_us;
    tr->cmd_len = req->cmd_len;
    /* Copy the ATN bytes into the fixed storage: the caller's buffer
     * may be stack-scoped (gpib_ppoll_configure uses a local array). */
    for (size_t ci = 0; ci < req->cmd_len && ci < GPIB_CFG_MAX_CMDS;
         ci++) {
        tr->cmd_stored[ci] = (uint8_t)(req->cmd_list[ci] & 0x7Fu);
    }
    tr->stop_cond = (uint8_t)req->stop_cond;
    tr->stop_dab = req->stop_dab;
    memset(&tr->result, 0, sizeof(tr->result));
    br->state = BLOCK_CORE_QUEUED;
    br->txn_slot = (uint16_t)t;
    br->txn_generation = tr->generation;
    br->txn_seq = tr->next_block_seq++; /* R17-01: FIFO order */
    /* First-block staging per kind; default-free for the same reason as
     * the validation switch above (new kinds must be staged explicitly). */
    switch (req->kind) {
    case GPIB_REQ_READ:
    case GPIB_REQ_DEV_RX:
        br->data = (const uint8_t *)req->rx_data;
        br->len = req->rx_cap;
        br->is_rx = true;
        break;
    case GPIB_REQ_SPOLL:
        br->data = &br->internal_payload; /* AUD-16: block-owned */
        br->len = 1;
        br->is_rx = true;
        break;
    case GPIB_REQ_TCT:
    case GPIB_REQ_CMDS:
        br->data = &br->internal_payload; /* dummy; no data phase runs */
        br->len = 1;
        br->is_rx = false;
        break;
    case GPIB_REQ_ALLSPOLL:
    case GPIB_REQ_FINDRQS:
        br->data = &br->internal_payload; /* dummy; the sequencer redirects
                                      per address into the results buffer */
        br->len = 1;
        br->is_rx = false;
        tr->poll_addrs = (const uint8_t *)req->tx_data;
        tr->poll_count = req->tx_len;
        tr->poll_index = 0;
        tr->poll_results = (uint8_t *)req->rx_data;
        tr->find_index = (size_t)-1;
        tr->find_out = req->find_out;
        break;
    case GPIB_REQ_RESET:
        br->data = &br->internal_payload; /* dummy; the sequencer drives *RST */
        br->len = 1;
        br->is_rx = false;
        tr->poll_addrs = (const uint8_t *)req->tx_data;
        tr->poll_count = req->tx_len;
        tr->poll_index = 0;
        /* A09: the bus-initialization side effects (REN + IFC, 488.2
         * 17.1 levels 1-2) belong to the ACCEPTED REQUEST, not to the
         * wrapper -- a raw gpib_submit(RESET) used to run DCL+*RST with
         * REN/IFC never asserted, reporting COMPLETED for a bus that
         * was never initialized. Arming here (post-reservation) keeps
         * rejected submits side-effect-free. */
        ctx->ren_request = true;
        ctx->ifc_request = true;
        tr->rst_msg[0] = '*';
        tr->rst_msg[1] = 'R';
        tr->rst_msg[2] = 'S';
        tr->rst_msg[3] = 'T';
        tr->rst_msg[4] = '\n';
        tr->rst_msg[5] = 0;
        break;
    case GPIB_REQ_PPOLL:
        br->data = &br->internal_payload; /* AUD-16: block-owned PPR */
        br->len = 1;
        br->is_rx = true;
        break;
    case GPIB_REQ_FINDLSTN:
        br->data = &br->internal_payload; /* dummy; sequencer writes probe results */
        br->len = 1;
        br->is_rx = false;
        tr->probe_addrs = (const uint8_t *)req->tx_data;
        tr->probe_results = (bool *)req->rx_data;
        tr->probe_count = req->tx_len;
        tr->probe_index = 0;
        break;
    case GPIB_REQ_WRITE:
    case GPIB_REQ_WRITE_MANY:
    case GPIB_REQ_DEV_TX:
        br->data = (const uint8_t *)req->tx_data;
        br->len = req->tx_len;
        br->is_rx = false;
        break;
    }
    memset(&br->result, 0, sizeof(br->result));
    *txn_out = gpib_make_handle(ctx, (uint8_t)GPIB_HANDLE_TXN, (uint16_t)t,
        tr->generation);
    *first_block_out = gpib_make_handle(ctx, (uint8_t)GPIB_HANDLE_BLOCK,
        (uint16_t)b, br->generation);
    GPIB_TRACE(ctx, GPIB_TRC_SUBMIT, (uint16_t)t, tr->generation, 0);
    return GPIB_OK;
}

gpib_status_t gpib_serial_poll(gpib_context_t *ctx, uint8_t address,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
    gpib_request_t r;
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_SPOLL;
    r.address.primary = address;
    r.address.secondary_present = false;
    return gpib_submit(ctx, &r, txn_out, block_out);
}

gpib_status_t gpib_allspoll(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, uint8_t *results, gpib_handle_t *txn_out,
    gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (addrs == NULL || results == NULL || n == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_ALLSPOLL;
    r.tx_data = addrs; /* borrow the address list through the request */
    r.tx_len = n;
    r.rx_data = results; /* borrow the results buffer */
    r.rx_cap = n;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)addrs;
    (void)n;
    (void)results;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_findrqs(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, uint8_t *results, size_t *found, gpib_handle_t *txn_out,
    gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (addrs == NULL || results == NULL || found == NULL || n == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_FINDRQS;
    r.tx_data = addrs;
    r.tx_len = n;
    r.rx_data = results;
    r.rx_cap = n;
    r.find_out = found;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)addrs;
    (void)n;
    (void)results;
    (void)found;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_system_reset(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, gpib_handle_t *txn_out, gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (!ctx->system_controller) {
        return GPIB_UNSUPPORTED; /* RESET level 1 needs C1+C2 */
    }
    if (addrs == NULL || n == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    /* Levels 1-2 run synchronously through the line engine: REN, IFC
     * pulse, DCL. Level 3 (*RST to each address) is queued as the
     * action's data phase. Since A09 the side effects are armed by the
     * submit path itself (post-reservation): the wrapper only builds
     * the request. */
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_RESET;
    r.tx_data = addrs; /* address list staged through the request */
    r.tx_len = n;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)addrs;
    (void)n;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_parallel_poll(gpib_context_t *ctx,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_PPOLL;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_ppoll_configure(gpib_context_t *ctx, uint8_t addr,
    uint8_t sense, uint8_t dio_line, gpib_handle_t *txn_out,
    gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    uint8_t cmds[5];
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (dio_line > 7u || sense > 1u) {
        return GPIB_INVALID_ARGUMENT;
    }
    /* A08: the primary address is validated like every other addressed
     * request -- gpib_mk_mla() masks to five bits, so an out-of-range
     * value used to silently configure ANOTHER device (addr 38 ->
     * MLA(6) on the wire). */
    if (addr > 30u) {
        return GPIB_INVALID_ARGUMENT;
    }
    cmds[0] = gpib_mk_mta(ctx->own_address.primary); /* MTA(ctrl) */
    cmds[1] = GPIB_UNL_BYTE;                                 /* UNL */
    cmds[2] = gpib_mk_mla(addr);                             /* MLA(dev) */
    cmds[3] = GPIB_PPC_BYTE;                                 /* PPC */
    cmds[4] = gpib_mk_ppe(sense, dio_line);                  /* PPE */
    return gpib_send_commands(ctx, cmds, 5, txn_out, block_out);
#else
    (void)ctx;
    (void)addr;
    (void)sense;
    (void)dio_line;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_ppoll_unconfigure(gpib_context_t *ctx,
    gpib_handle_t *txn_out, gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    uint8_t ppu[1] = { GPIB_PPU_BYTE };
    if (ctx == NULL || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    return gpib_send_commands(ctx, ppu, 1, txn_out, block_out);
#else
    (void)ctx;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_findlstn(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, bool *results, gpib_handle_t *txn_out,
    gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    gpib_request_t r;
    if (ctx == NULL || txn_out == NULL || block_out == NULL ||
        addrs == NULL || results == NULL || n == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    memset(&r, 0, sizeof(r));
    r.kind = GPIB_REQ_FINDLSTN;
    r.tx_data = addrs;
    r.tx_len = n;
    r.rx_data = results;
    r.rx_cap = n;
    return gpib_submit(ctx, &r, txn_out, block_out);
#else
    (void)ctx;
    (void)addrs;
    (void)n;
    (void)results;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_request_control(gpib_context_t *ctx) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    /* REQUESTCTL (17.5) requesting side: assert SRQ. The CIC will
     * serial-poll us, find RQS, and pass control via TCT. */
    ctx->device_rsv = true;
    return GPIB_OK;
#else
    (void)ctx;
    return GPIB_UNSUPPORTED;
#endif
}

/* REQUESTCTL CIC-side candidate bound: the composite stages poll results
 * on its own stack, so the list is capped (a caller needing more should
 * drive gpib_findrqs + gpib_pass_control directly). */
#define REQUESTCTL_MAX_ADDRS 16u

gpib_status_t gpib_requestctl_pass(gpib_context_t *ctx, const uint8_t *addrs,
    size_t n, size_t *found, gpib_handle_t *txn_out,
    gpib_handle_t *block_out) {
#if GPIB_CFG_ROLE == GPIB_ROLE_CONTROLLER
    /* CIC side of REQUESTCTL: FINDRQS then TCT to the requester.
     * R6-04: the results buffer is INSTANCE memory, never a local --
     * even a port that wedges past our bounded drain writes into a
     * valid object, not a dead stack frame. */
    gpib_handle_t fq_t = { 0, 0, 0, 0, 0 };
    gpib_handle_t fq_b = { 0, 0, 0, 0, 0 };
    gpib_result_t fq_res;
    int guard = 0;
    if (ctx == NULL || addrs == NULL || found == NULL || n == 0 ||
        n > REQUESTCTL_MAX_ADDRS || txn_out == NULL || block_out == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    *found = (size_t)-1;
    if (gpib_findrqs(ctx, addrs, n, ctx->requestctl_results, found,
            &fq_t, &fq_b) != GPIB_OK) {
        return GPIB_INVALID_STATE;
    }
    /* CONTRACT: this composite BLOCKS inside the call -- it pumps
     * gpib_process() until the internal FINDRQS finishes (bounded by the
     * guard). It is the one synchronous convenience in an otherwise
     * asynchronous API: for non-blocking use, drive gpib_findrqs and
     * gpib_pass_control directly. */
    while (gpib_result_peek(ctx, fq_t, &fq_res) != GPIB_OK && guard++ < 40000) {
        gpib_process(ctx);
    }
    if (guard >= 40000) {
        /* F06: the FINDRQS poll_results buffer is INSTANCE memory
         * (ctx->requestctl_results, R6-04) -- an unowned RUNNING action
         * would keep writing into it after the return. Cancel the
         * transaction and drain it to a terminal state before leaving;
         * if even the drain wedges (stuck port), the action is
         * stop-latched, not polling. */
        (void)gpib_request_cancel(ctx, fq_t);
        int drain = 0;
        while (gpib_result_peek(ctx, fq_t, &fq_res) != GPIB_OK &&
               drain++ < 40000) {
            gpib_process(ctx);
        }
        /* Rev10 §6.2: the ACK results are CHECKED, not ignored -- a
         * failed ACK means the ledger slots are still held; surface it
         * in the trace so a wedged drain is diagnosable instead of
         * silently leaking slots until deinit. */
        gpib_status_t ack_b = gpib_result_ack(ctx, fq_b);
        gpib_status_t ack_t = gpib_result_ack(ctx, fq_t);
        if (ack_b != GPIB_OK || ack_t != GPIB_OK) {
            GPIB_TRACE(ctx, GPIB_TRC_ACCOUNT, 0xFFFFu, 0xFFFFu,
                (uint32_t)((ack_t << 8u) | (ack_b & 0xFFu)));
        }
        return GPIB_BUSY_RESOURCES;
    }
    /* Free the FINDRQS ledger slots: its result (found index) is already
     * consumed here; the caller's follow-up needs the slots. */
    gpib_result_ack(ctx, fq_b);
    gpib_result_ack(ctx, fq_t);
    if (*found == (size_t)-1) {
        return GPIB_NOT_FOUND; /* nobody requesting service at all */
    }
    /* AUD-15 (488.2 17.5.5): a generic RQS is NOT a request for
     * control -- it may be MAV, an error or a completed measurement.
     * The full REQUESTCTL protocol distinguishes RQC (ESR bit + the
     * request-control path with a return address via the PCB
     * convention) from ordinary service requests; that requires
     * device-side semantics this core does not own. The composite
     * therefore STOPS at the poll: `*found` names the first requester
     * and its STB is in the results buffer -- the APPLICATION decides
     * (and, if it really is RQC, issues gpib_pass_control itself).
     * Blindly TCT-ing the first RQS source used to strip CIC from
     * this controller on any instrument's data-ready. */
    return GPIB_OK;
#else
    (void)ctx;
    (void)addrs;
    (void)n;
    (void)found;
    (void)txn_out;
    (void)block_out;
    return GPIB_UNSUPPORTED;
#endif
}

gpib_status_t gpib_submit_block(gpib_context_t *ctx, gpib_handle_t txn,
    const void *data, size_t len, gpib_handle_t *block_out) {
    /* N12-02 (rev12): a zero-length block reaches the executor with
     * nothing to transfer and stalls it forever (SH parks in SGNS with
     * no byte; the transaction never terminalizes). The zero-length
     * POLICY stays open (OPEN-23) -- until it is defined, the API
     * rejects len==0 in BOTH directions, exactly like the first block.
     * A zero-length terminal marker, if ever needed, must be its own
     * action kind, never a 0-byte TX/RX fed to the FSMs. */
    if (ctx == NULL || data == NULL || block_out == NULL || len == 0) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    int s = gpib_ledger_txn_slot(ctx, txn);
    if (s < 0) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->txn[s].state != TXN_OPEN) {
        return GPIB_INVALID_STATE; /* transaction no longer accepts blocks */
    }
    int b = gpib_ledger_find_free_block(ctx);
    if (b < 0) {
        return GPIB_BUSY_RESOURCES;
    }
    struct gpib_txn_record *tr = &ctx->txn[s];
    struct gpib_block_record *br = &ctx->block[b];
    br->state = BLOCK_CORE_QUEUED;
    br->txn_slot = (uint16_t)s;
    br->txn_generation = tr->generation;
    br->txn_seq = tr->next_block_seq++; /* R17-01: FIFO order */
    br->data = (const uint8_t *)data;
    br->len = len;
    br->is_rx = (tr->kind == GPIB_REQ_READ ||
                 tr->kind == GPIB_REQ_DEV_RX); /* consistent with first block */
    memset(&br->result, 0, sizeof(br->result));
    tr->block_count++;
    *block_out = gpib_make_handle(ctx, (uint8_t)GPIB_HANDLE_BLOCK,
        (uint16_t)b, br->generation);
    GPIB_TRACE(ctx, GPIB_TRC_SUBMIT_BLOCK, (uint16_t)b, (uint16_t)s, 0);
    return GPIB_OK;
}

gpib_status_t gpib_request_cancel(gpib_context_t *ctx, gpib_handle_t txn) {
    if (ctx == NULL) {
        return GPIB_INVALID_ARGUMENT;
    }
    if (ctx->lifecycle == GPIB_LC_UNINITIALIZED) {
        return GPIB_INVALID_HANDLE;
    }
    if (ctx->lifecycle == GPIB_LC_STOPPING) {
        return GPIB_SHUTDOWN_IN_PROGRESS;
    }
    if (ctx->lifecycle != GPIB_LC_RUNNING) {
        return GPIB_INVALID_STATE;
    }
    int s = gpib_ledger_txn_slot(ctx, txn);
    if (s < 0) {
        return GPIB_NOT_FOUND;
    }
    struct gpib_txn_record *t = &ctx->txn[s];
    if (t->state == TXN_RESULT_READY) {
        return GPIB_ALREADY_TERMINAL;
    }
    /* Dispatched? Then the stop latch ends the action at a phase boundary
     * and the terminal record appears after quiescence (async cancel);
     * never-dispatched blocks take the shortcut (FIX-02). */
    bool dispatched = false;
    for (int i = 0; i < GPIB_CFG_MAX_BLOCKS; i++) {
        struct gpib_block_record *b = &ctx->block[i];
        if (b->txn_slot == (uint16_t)s && b->txn_generation == t->generation &&
            b->state == BLOCK_EXECUTOR_IN_USE) {
            dispatched = true;
            break;
        }
    }
    if (dispatched) {
        if (!ctx->stop_request.active) {
            ctx->stop_request.active = true;
            ctx->stop_request.cause = (uint8_t)GPIB_TERM_CANCELED;
            ctx->stop_request.txn_slot = (uint16_t)s;
        }
        GPIB_TRACE(ctx, GPIB_TRC_CANCEL, (uint16_t)s, t->generation, 1u);
        return GPIB_OK; /* local, asynchronous; instance stays RUNNING */
    }
    /* Local, synchronous shortcut: the instance stays RUNNING (LIF-05); the
     * transaction terminal record follows after all blocks are terminal. */
    if (ctx->line.rx_session_open &&
        ctx->line.rx_session_slot == (uint16_t)s &&
        ctx->line.rx_session_gen == t->generation) {
        /* C5: the open RECEIVE dies with its transaction -- the next READ
         * must run a fresh RECEIVE SETUP (asserting ATN takes the bus
         * back from the old talker), never resume this stream. */
        ctx->line.rx_session_open = false;
    }
    t->state = TXN_FINISHING;
    gpib_ledger_terminalize_txn_blocks(ctx, (uint16_t)s);
    gpib_ledger_try_publish_txn(ctx, (uint16_t)s, GPIB_OK, GPIB_TERM_CANCELED,
        GPIB_END_NONE);
    GPIB_TRACE(ctx, GPIB_TRC_CANCEL, (uint16_t)s, t->generation, 0);
    return GPIB_OK;
}
