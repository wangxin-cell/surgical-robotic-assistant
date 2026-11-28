param([Parameter(Mandatory)][string]$Episode, [int]$Offset=0, [string]$Pose='original')
$ErrorActionPreference = 'Stop'
[Console]::InputEncoding = [Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$source = Join-Path $PSScriptRoot 'BimanualArmTeach.ps1'
if ((Get-FileHash -Algorithm SHA256 $source).Hash -ne '1FE2A3E524AE1B25BD8BC91944E8A9B307C25A514F8AAE21EEDE0EC3DFB6F708') {
    throw 'Original replay script checksum mismatch'
}
# Insert only platform I/O bindings before the original forward-replay branch.
# Its validation, Set-Goal, command ordering and timing calculation run verbatim.
$program = [IO.File]::ReadAllText($source)
$marker = 'if ($ReplayEpisode) {'
$entry = $program.IndexOf($marker,[StringComparison]::Ordinal)
if ($entry -lt 0 -or $entry -ne $program.LastIndexOf($marker,[StringComparison]::Ordinal)) {
    throw 'Cannot identify forward replay entry'
}
$bindings = @'
function Exchange([string]$line) {
    [Console]::Out.WriteLine("BUS $line")
    $reply = [Console]::In.ReadLine()
    if ($null -eq $reply -or !$reply.StartsWith('OK ')) { throw 'O6 serial bridge stopped' }
    return $reply.Substring(3)
}
function Open-Bus { $null = Exchange 'open' }
function Read-Register([int]$id,[int]$address,[int]$count) {
    $bytes = Exchange "read $id $address $count"
    return ,([Convert]::FromBase64String($bytes))
}
function Send-Packet([int]$id,[int]$opcode,[int]$address,[byte[]]$payload) {
    $null = Exchange "write $id $opcode $address $([Convert]::ToBase64String($payload))"
}
function Read-Host($Prompt) { return (Exchange 'confirm') }
function Start-Sleep([int]$Milliseconds) { $null = Exchange "wait $Milliseconds" }
'@
$before = $program.Substring(0,$entry)
$forward = $program.Substring($entry)
if ($Pose -ne 'original') {
    $positions = @($Pose.Split(',') | ForEach-Object { [int]$_ })
    if ($positions.Count -ne 6) { throw 'Incomplete resume pose' }
    $at = $forward.IndexOf('    if (-not $episodeActions.Count')
    $forward = $forward.Insert($at,'    foreach ($joint in $firstPose) { $joint.position = $positions[$joint.id-1] }' + [Environment]::NewLine)
}
if ($Offset -gt 0) {
    $marker = '$prevTime = [DateTimeOffset]::Parse($episodeActions[0].time_utc)'
    $at = $forward.IndexOf($marker) + $marker.Length
    $forward = $forward.Insert($at,[Environment]::NewLine + '        $prevTime = [DateTimeOffset]::Parse($episodeActions[$Offset-1].time_utc)
        $episodeActions = @($episodeActions | Select-Object -Skip $Offset)' + [Environment]::NewLine)
}
& ([scriptblock]::Create($before + $bindings + [Environment]::NewLine + $forward)) -Arm left -ReplayEpisode $Episode -ExecuteReplay

