$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
$rel = Invoke-RestMethod "https://api.github.com/repos/oschwartz10612/poppler-windows/releases/latest"
$asset = $rel.assets | Where-Object { $_.name -match "zip$" } | Select-Object -First 1
"asset: " + $asset.name
$tmp = "F:\Download\gpib-core\build\poppler.zip"
curl.exe -L --connect-timeout 30 --max-time 300 -o $tmp $asset.browser_download_url
if ($LASTEXITCODE -ne 0) { exit 1 }
"dl: " + (Get-Item $tmp).Length
Expand-Archive -Path $tmp -DestinationPath "F:\Download\gpib-core\build\poppler" -Force
$pt = (Get-ChildItem "F:\Download\gpib-core\build\poppler" -Recurse -Filter "pdftotext.exe" | Select-Object -First 1).FullName
"pdftotext: $pt"
& $pt -layout "F:\Download\IEEE-488.2-1992.pdf" "F:\Download\IEEE-488.2-1992.txt"
"exit: $LASTEXITCODE"
if (Test-Path "F:\Download\IEEE-488.2-1992.txt") { "txt: " + (Get-Item "F:\Download\IEEE-488.2-1992.txt").Length }
