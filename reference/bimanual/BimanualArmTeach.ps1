param([ValidateSet('left','right')][string]$Arm='left', [switch]$Simulate, [switch]$CheckOnly, [switch]$SelfTest)

$ErrorActionPreference = 'Stop'
$ids = 1..6
$serial = if ($Arm -eq 'right') { '5AAF288004' } else { '5A7C122569' }
$margin = 24
$script:bus = $null
$script:holding = $false
$script:locked = $false
$script:lastJog = @{}
$script:pending = @{}
$traceFile = Join-Path $PSScriptRoot 'bimanual-arm-diagnostics.jsonl'
$script:simState = @{}
$simLimits = @(@(1204,3040),@(993,3248),@(837,3053),@(911,3234),@(47,3938),@(1391,2953))
$simPositions = @(2336,2311,1367,2771,1082,1572)
foreach ($id in $ids) {
    $script:simState[$id] = @{Min=$simLimits[$id-1][0]; Max=$simLimits[$id-1][1];
        Pos=$simPositions[$id-1]; Torque=0; Goal=0}
}
if ($SelfTest) { $Simulate = $true }

function Open-Bus {
    if ($Simulate) { return 'SIMULATION' }
    if ($script:bus -and $script:bus.IsOpen) { return $script:bus.PortName }
    $key = "HKLM:\SYSTEM\CurrentControlSet\Enum\USB\VID_1A86&PID_55D3\$serial\Device Parameters"
    $name = (Get-ItemProperty -LiteralPath $key).PortName
    if ($name -notmatch '^COM[0-9]+$' -or $name -notin [IO.Ports.SerialPort]::GetPortNames()) {
        throw 'This left-arm controller is not connected.'
    }
    $port = [IO.Ports.SerialPort]::new($name,1000000,'None',8,'One')
    $port.ReadTimeout = 25
    $port.WriteTimeout = 100
    try { $port.Open() } catch { $port.Dispose(); throw }
    $script:bus = $port
    return $name
}

function Send-Packet([int]$id, [int]$opcode, [int]$address, [byte[]]$payload) {
    if ($id -notin $ids) { throw 'Motor is not on the left-arm allowlist.' }
    [byte[]]$body = @($id, ($payload.Length+3), $opcode, $address) + $payload
    $sum = 0
    foreach ($value in $body) { $sum += $value }
    [byte[]]$packet = @(255,255) + $body + @((-bnot $sum) -band 255)
    $script:bus.DiscardInBuffer()
    $script:bus.Write($packet,0,$packet.Length)
    return ,$packet
}

function Read-Register([int]$id, [int]$address, [int]$count) {
    if ($Simulate) {
        $s = $script:simState[$id]
        switch ($address) {
            9  { return ,([byte[]]@(($s.Min-band 255),($s.Min-shr 8),($s.Max-band 255),($s.Max-shr 8))) }
            33 { return ,([byte[]]@(0)) }
            40 { return ,([byte[]]@($s.Torque)) }
            42 { return ,([byte[]]@(($s.Goal-band 255),($s.Goal-shr 8))) }
            56 { return ,([byte[]]@(($s.Pos-band 255),($s.Pos-shr 8))) }
            60 { return ,([byte[]]@(0,0,125)) }
            66 { return ,([byte[]]@(0)) }
            default { throw "Unsupported simulation register $address" }
        }
    }
    $request = Send-Packet $id 2 $address @($count)
    $size = $count + 6
    $buffer = [Collections.Generic.List[int]]::new()
    $clock = [Diagnostics.Stopwatch]::StartNew()
    while ($clock.ElapsedMilliseconds -lt 110 -and $buffer.Count -lt 80) {
        try { $buffer.Add($script:bus.ReadByte()) } catch {
            if ($_.Exception.GetBaseException() -is [TimeoutException]) { continue }
            throw
        }
        if ($buffer.Count -lt $size) { continue }
        $frame = $buffer.GetRange($buffer.Count-$size,$size)
        if ($frame[0] -ne 255 -or $frame[1] -ne 255 -or $frame[2] -ne $id -or $frame[3] -ne ($count+2)) { continue }
        if (($frame -join ',') -eq ($request -join ',')) { continue }
        $sum = 0
        for ($i=2; $i -lt $size; $i++) { $sum += $frame[$i] }
        if (($sum -band 255) -ne 255) { continue }
        if ($frame[4] -ne 0) { throw "ID ${id}: servo status $($frame[4])" }
        return ,([byte[]]$frame.GetRange(5,$count).ToArray())
    }
    throw "ID ${id}: no valid reply for register $address"
}

function Word($bytes,[int]$offset=0) { return ([int]$bytes[$offset]+256*[int]$bytes[$offset+1]) }
function Read-Arm {
    foreach ($id in $ids) {
        $limits = Read-Register $id 9 4
        [pscustomobject]@{
            id=$id; position=(Word (Read-Register $id 56 2))
            torque=(Read-Register $id 40 1)[0]; mode=(Read-Register $id 33 1)[0]
            goal=(Word (Read-Register $id 42 2))
            min=(Word $limits); max=(Word $limits 2)
        }
    }
}

function Assert-Position($joint, [int]$position) {
    if ($joint.mode -ne 0 -or $joint.min -lt 0 -or $joint.min -ge $joint.max -or $joint.max -gt 4095) {
        throw "ID $($joint.id)：位置模式或配置限位异常。"
    }
    if ($position -lt $joint.min -or $position -gt $joint.max) {
        throw "ID $($joint.id)：实测 $position 超出舵机限位 $($joint.min)–$($joint.max)，先支撑并释放再核对姿态。"
    }
}

function Get-JogTarget($joint,[int]$delta) {
    $target = $joint.position+$delta
    $low = $joint.min+$margin
    $high = $joint.max-$margin
    if ($target -ge $low -and $target -le $high) { return $target }
    if ($joint.position -ge $low -and $joint.position -le $high) {
        # Shorten the last step instead of rejecting the entire click.
        $bounded = [Math]::Min($high,[Math]::Max($low,$target))
        if ($bounded -ne $joint.position) { return $bounded }
    }
    if ($target -gt $joint.min -and $target -lt $joint.max -and
        (($joint.position -lt $low -and $delta -gt 0 -and $target -le $high) -or
         ($joint.position -gt $high -and $delta -lt 0 -and $target -ge $low))) {
        return $target
    }
    $direction = if ($target -lt $low) { '+' } else { '−' }
    throw "点动拦截：ID $($joint.id) 目标 $target 靠近限位。请按 $direction 往范围内移。"
}

function Write-Diagnostic([string]$eventName,[hashtable]$fields) {
    $fields.event = $eventName
    $fields.time = [DateTimeOffset]::Now.ToString('o')
    $fields.simulation = [bool]$Simulate
    try {
        $line = $fields | ConvertTo-Json -Depth 5 -Compress
        [IO.File]::AppendAllText($traceFile,$line+[Environment]::NewLine,[Text.UTF8Encoding]::new($false))
    } catch {
        if ($status) { $status.Text = "诊断日志写入失败：$($_.Exception.Message)" }
    }
}

function Read-JointFeedback([int]$id) {
    $feedback = Read-Register $id 60 3
    return @{
        position=(Word (Read-Register $id 56 2))
        goal=(Word (Read-Register $id 42 2))
        moving=(Read-Register $id 66 1)[0]
        load_raw=(Word $feedback)
        voltage_v=([int]$feedback[2]/10.0)
    }
}

function Set-Torque([int]$id, [int]$enabled) {
    if ($Simulate) { $script:simState[$id].Torque = $enabled; return }
    $null = Send-Packet $id 3 40 @($enabled)
}

function Set-Goal([int]$id, [int]$position) {
    if ($Simulate) {
        $script:simState[$id].Goal = $position
        $script:simState[$id].Pos = $position
        $script:simState[$id].Torque = 1
    } else {
        $null = Send-Packet $id 3 41 @(5,($position-band 255),($position-shr 8),0,0,45,0)
    }
    if ((Word (Read-Register $id 42 2)) -ne $position) { throw "ID ${id}: target write could not be verified." }
}

function Enable-Hold {
    if ($script:locked -or $script:holding) { throw 'Hold is not available in the current state.' }
    $joints = @(Read-Arm)
    foreach ($joint in $joints) {
        Assert-Position $joint $joint.position
        if ($joint.torque -ne 0) { throw "ID $($joint.id) already has torque. Support the arm, then release and reconnect." }
    }
    $script:locked = $true
    foreach ($joint in $joints) {
        $id = $joint.id
        $position = Word (Read-Register $id 56 2)
        Assert-Position $joint $position
        if ([Math]::Abs($position-$joint.position) -gt 5) { throw "ID ${id}: pose moved during engagement." }
        Set-Goal $id $position
        if ((Read-Register $id 40 1)[0] -eq 0) { Set-Torque $id 1 }
        if ((Read-Register $id 40 1)[0] -ne 1) { throw "ID ${id}: hold could not be verified." }
        if ([Math]::Abs((Word (Read-Register $id 56 2))-$position) -gt 8) { throw "ID ${id}: motion detected during engagement." }
    }
    $script:holding = $true
    $script:lastJog.Clear()
    $script:locked = $false
}

function Jog-Joint([int]$id,[int]$delta) {
    if (-not $script:holding -or $script:locked -or $id -notin $ids -or
            [Math]::Abs($delta) -notin @(8,16,24)) { throw 'Jog is unavailable.' }
    $joints = @(Read-Arm)
    foreach ($item in $joints) {
        Assert-Position $item $item.position
        if ($item.torque -ne 1) { throw "ID $($item.id): unexpected loss of hold." }
    }
    $joint = $joints[$id-1]
    $lag = $joint.goal-$joint.position
    if ([Math]::Abs($lag) -gt 12) {
        $moving = (Read-Register $id 66 1)[0]
        if ($moving) { throw "关节还在移动：ID ${id}。等实测位置稳定后再点。" }
        $previous = $script:lastJog[$id]
        $progressed = $previous -and $previous.goal -eq $joint.goal -and
            ($joint.position-$previous.start)*[Math]::Sign($previous.goal-$previous.start) -ge 4
        $cancel = $lag*$delta -lt 0
        $reverse = if ($lag -gt 0) { '−' } else { '+' }
        $reason = if ([Math]::Abs($lag) -le 24) {
            "反馈停在旧目标之外。若无异响或卡碰，可按 $reverse 取消旧目标。"
        } else {
            '反馈停在旧目标之外，偏差超过单步范围。请支撑臂身并检查。'
        }
        if ([Math]::Abs($lag) -le 24 -and ($cancel -or $progressed)) {
            Start-Sleep -Milliseconds 120
            $position = Word (Read-Register $id 56 2)
            if ((Read-Register $id 66 1)[0] -ne 0 -or [Math]::Abs($position-$joint.position) -gt 2) {
                throw "点动拦截：ID ${id} 尚未停稳，请等实测位置稳定。"
            }
            Assert-Position $joint $position
            if ($cancel) {
                Set-Goal $id $position
                $script:lastJog.Remove($id)
                return [pscustomobject]@{position=$position; reset=$true}
            }
            # Rebase the next bounded step on fresh feedback, never on the old goal.
            $joint.position = $position
        } else {
            throw "点动拦截：ID ${id} 实测 $($joint.position)，上次目标 $($joint.goal)。$reason"
        }
    }
    $target = Get-JogTarget $joint $delta
    Set-Goal $id $target
    $script:lastJog[$id] = @{start=$joint.position; goal=$target}
    return [pscustomobject]@{position=$target; reset=$false}
}

function Release-Arm {
    $script:holding = $false
    $script:lastJog.Clear()
    $script:pending.Clear()
    $script:locked = $true
    $failures = @()
    foreach ($id in $ids) {
        try {
            Set-Torque $id 0
            if ((Read-Register $id 40 1)[0] -ne 0) { throw 'Release unconfirmed' }
        } catch { $failures += $id }
    }
    if ($failures.Count) { throw "Release unconfirmed for IDs $($failures -join ', '). Use the physical power cutoff with the arm supported." }
    $script:locked = $false
}

function Save-Pose([string]$path,[string]$name,[object[]]$joints) {
    [ordered]@{
        schema=1; arm=$Arm; controller_serial=$serial; captured_at=[DateTimeOffset]::Now.ToString('o')
        label=$name; simulation=[bool]$Simulate; units='encoder_counts'; motion_validated=$false
        joints=$joints
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $path -Encoding UTF8
}

if ($CheckOnly) {
    try { Open-Bus; Read-Arm | Format-Table -AutoSize } finally { if ($script:bus) { $script:bus.Dispose() } }
    return
}
if ($SelfTest) {
    Enable-Hold
    if ((Jog-Joint 6 16).position -ne 1588) { throw 'Unexpected motion target' }
    $sample = @(Read-Arm)
    if ($sample[5].position -ne 1588 -or $sample[0].position -ne 2336) { throw 'Unexpected simulated position' }
    $script:simState[6].Pos = 2928
    $script:simState[6].Goal = 2928
    if ((Jog-Joint 6 16).position -ne 2929) { throw 'Shortened upper limit step failed' }
    $blocked = $false
    try { $null = Jog-Joint 6 16 } catch { $blocked = $true }
    if (-not $blocked -or $script:simState[6].Goal -ne 2929) { throw 'Limit guard failed' }
    $script:simState[4].Pos = 3223
    $script:simState[4].Goal = 3223
    $blocked = $false
    try { $null = Jog-Joint 4 8 } catch { $blocked = $true }
    if (-not $blocked -or $script:simState[4].Goal -ne 3223) { throw 'Outward guard failed' }
    if ((Jog-Joint 4 -8).position -ne 3215 -or (Jog-Joint 4 -8).position -ne 3207) { throw 'Inward recovery failed' }
    $script:simState[3].Goal = $script:simState[3].Pos + 30
    $blocked = $false
    try { $null = Jog-Joint 3 8 } catch { $blocked = $_.Exception.Message -like '*反馈停在*' }
    if (-not $blocked) { throw 'Stalled feedback was not identified' }
    $script:simState[3].Goal = $script:simState[3].Pos + 16
    $blocked = $false
    try { $null = Jog-Joint 3 8 } catch { $blocked = $_.Exception.Message -like '*可按*取消旧目标*' }
    if (-not $blocked -or $script:simState[3].Goal -ne ($script:simState[3].Pos+16)) {
        throw 'Same-direction protection failed.'
    }
    $reset = Jog-Joint 3 -8
    if (-not $reset.reset -or $script:simState[3].Goal -ne $script:simState[3].Pos) {
        throw 'Reverse reset failed to cancel the stalled target.'
    }
    $script:simState[4].Pos = $script:simState[4].Goal = 2792
    if ((Jog-Joint 4 24).position -ne 2816) { throw 'Failed to issue the first jog' }
    $script:simState[4].Pos = 2800
    if ((Jog-Joint 4 8).position -ne 2808) { throw 'Stable partial movement was not rebased' }
    $script:simState[4].Pos = $script:simState[4].Goal = 2792
    $null = Jog-Joint 4 24
    $script:simState[4].Pos = 2792
    $blocked = $false
    try { $null = Jog-Joint 4 24 } catch { $blocked = $_.Exception.Message -like '*反馈停在*' }
    if (-not $blocked -or $script:simState[4].Goal -ne 2816) { throw 'No-progress guard failed' }
    $script:simState[2].Pos = $script:simState[2].Goal = 1022
    if ((Jog-Joint 2 -8).position -ne 1017) { throw 'Shortened limit step failed' }
    $blocked = $false
    try { $null = Jog-Joint 2 -8 } catch { $blocked = $true }
    if (-not $blocked -or $script:simState[2].Goal -ne 1017) { throw 'Software limit was crossed' }
    if ((Jog-Joint 2 8).position -ne 1025) { throw 'Movement away from the limit failed' }
    if ((Read-JointFeedback 2).position -ne 1025) { throw 'Diagnostic feedback failed' }
    Release-Arm
    if (@(Read-Arm | Where-Object torque -ne 0).Count) { throw 'Release guard failed' }
    $temp = [IO.Path]::GetTempFileName()
    try {
        Save-Pose $temp 'A_hover' $sample
        $saved = Get-Content -LiteralPath $temp -Raw | ConvertFrom-Json
        if ($saved.joints.Count -ne 6 -or -not $saved.simulation) { throw 'Sample export failed' }
    } finally { Remove-Item -LiteralPath $temp }
    'Self-test passed: hold, isolated jog, limit rejection, release, pose export.'
    return
}

Add-Type -AssemblyName System.Windows.Forms,System.Drawing
[Windows.Forms.Application]::EnableVisualStyles()
$form = [Windows.Forms.Form]::new()
$form.Text = if ($Simulate) { '双臂单臂点动 v1.5（模拟）' } else { '双臂单臂点动 v1.5 · $Arm · $serial' }
$form.StartPosition = 'CenterScreen'
$form.ClientSize = [Drawing.Size]::new(740, 525)
$form.FormBorderStyle = 'FixedDialog'
$form.MaximizeBox = $false
$form.Font = [Drawing.Font]::new('Microsoft YaHei UI', 10)

function Add-Label($text,$x,$y,$width,$height=25) {
    $control = [Windows.Forms.Label]::new()
    $control.Text = $text
    $control.SetBounds($x,$y,$width,$height)
    $form.Controls.Add($control)
    return $control
}

function Add-Button($text,$x,$y,$width) {
    $control = [Windows.Forms.Button]::new()
    $control.Text = $text
    $control.SetBounds($x,$y,$width,32)
    $form.Controls.Add($control)
    return $control
}

$connect = Add-Button '连接并读取' 20 18 115
$hold = Add-Button '启用保持' 145 18 110
$release = Add-Button '支撑后释放' 265 18 120
$support = [Windows.Forms.CheckBox]::new()
$support.Text = '臂身已支撑，夹爪空置，手已移开'
$support.SetBounds(401,18,310,32)
$form.Controls.Add($support)
$null = Add-Label '步长（编码器计数）' 20 66 155
$step = [Windows.Forms.ComboBox]::new()
$step.DropDownStyle = 'DropDownList'
$step.SetBounds(178,63,74,28)
$step.Items.AddRange([object[]]@('8','16','24'))
$step.SelectedIndex = 0
$form.Controls.Add($step)
$null = Add-Label 'ID1–6：肩旋、肩升、肘、腕弯、腕旋、夹爪；夹爪方向需现场确认。' 272 66 450

$null = Add-Label '关节' 23 107 100
$null = Add-Label '实测' 138 107 70
$null = Add-Label '目标' 213 107 62
$null = Add-Label '配置限位' 293 107 130
$null = Add-Label '力矩' 438 107 70
$null = Add-Label '单步点动' 548 107 135
$rows = @{}
foreach ($id in $ids) {
    $y = 132+40*($id-1)
    $name = if ($id -eq 6) { 'ID 6  夹爪' } else { "ID $id" }
    $null = Add-Label $name 23 ($y+5) 100
    $position = Add-Label '—' 138 ($y+5) 70
    $target = Add-Label '—' 213 ($y+5) 70
    $limits = Add-Label '—' 293 ($y+5) 130
    $torque = Add-Label '—' 438 ($y+5) 68
    $minus = Add-Button '−' 548 $y 64
    $plus = Add-Button '+' 628 $y 64
    $minus.Tag = @{id=$id; sign=-1}
    $plus.Tag = @{id=$id; sign=1}
    $rows[$id] = @{position=$position; target=$target; limits=$limits; torque=$torque; minus=$minus; plus=$plus}
}

$null = Add-Label '姿态名称' 23 392 90
$poseName = [Windows.Forms.TextBox]::new()
$poseName.Text = 'A_hover'
$poseName.SetBounds(118,389,274,29)
$form.Controls.Add($poseName)
$save = Add-Button '保存实测姿态…' 414 386 180
$status = Add-Label '未连接。程序启动时力矩和位置均不写入。' 23 439 694 65
$script:connected = $false
$timer = [Windows.Forms.Timer]::new()
$timer.Interval = 1000

function Update-Buttons {
    $hold.Enabled = $script:connected -and -not $script:locked -and -not $script:holding -and $support.Checked
    $release.Enabled = $script:connected -and $support.Checked
    $save.Enabled = $script:connected -and -not $script:locked
    foreach ($row in $rows.Values) {
        $row.plus.Enabled = $row.minus.Enabled = $script:holding -and -not $script:locked
    }
}

function Show-Arm {
    foreach ($joint in @(Read-Arm)) {
        $row = $rows[$joint.id]
        $row.position.Text = [string]$joint.position
        $row.target.Text = if ($joint.torque -eq 1) { [string]$joint.goal } else { '—' }
        $row.limits.Text = "$($joint.min) – $($joint.max)"
        $row.torque.Text = if ($joint.torque -eq 0) { '关闭' } else { '保持' }
        $near = $joint.position -lt ($joint.min+$margin) -or $joint.position -gt ($joint.max-$margin)
        $row.position.ForeColor = if ($near) { [Drawing.Color]::Firebrick } else { [Drawing.Color]::Black }
        if ($script:holding -and $joint.torque -ne 1) { throw "ID $($joint.id): torque was lost." }
    }
}

function Show-Fault($errorObject) {
    if ($script:holding -or $script:locked) { $script:locked = $true }
    $status.Text = "操作中止：$($errorObject.Exception.Message)"
    Update-Buttons
}

$connect.Add_Click({
    try {
        $portName = Open-Bus
        Show-Arm
        $script:connected = $true
        $status.Text = "$portName 已连接。先看读数，再决定是否启用保持。"
        $timer.Start()
    } catch { Show-Fault $_ }
    Update-Buttons
})
$support.Add_CheckedChanged({ Update-Buttons })
$hold.Add_Click({
    if (-not $support.Checked) { return }
    try {
        Enable-Hold
        Show-Arm
        $status.Text = '已验证各关节保持。点 + 或 − 每次只移动一个关节。'
    } catch { Show-Fault $_ }
    Update-Buttons
})
$release.Add_Click({
    if (-not $support.Checked) { return }
    $answer = [Windows.Forms.MessageBox]::Show(
        '确认臂身已经承托。释放后手臂可能随重力下落。', '释放力矩',
        'OKCancel', 'Warning')
    if ($answer -ne 'OK') { return }
    try {
        Release-Arm
        Show-Arm
        $support.Checked = $false
        $status.Text = '已读回确认六个关节力矩关闭。'
    } catch { Show-Fault $_ }
    Update-Buttons
})

$jogHandler = {
    param($sender,$eventArgs)
    $id = $sender.Tag.id
    $delta = [int]$step.SelectedItem * $sender.Tag.sign
    $before = $null
    try {
        $before = Read-JointFeedback $id
        $goal = Jog-Joint $id $delta
        Write-Diagnostic 'accepted' @{id=$id; delta=$delta; before=$before; target=$goal.position; reset=$goal.reset}
        $script:pending[$id] = @{target=$goal.position; after=[DateTimeOffset]::Now.AddMilliseconds(700)}
        $status.Text = if ($goal.reset) {
            "ID $id 已取消旧目标，保持当前位置 $($goal.position)。需要后退时再按一次反方向。"
        } else {
            "ID $id 已下发目标 $($goal.position)。请看实测位置确认运动。"
        }
    } catch {
        Write-Diagnostic 'rejected_or_error' @{id=$id; delta=$delta; before=$before; reason=$_.Exception.Message}
        if ($_.Exception.Message -like '点动拦截：*' -or $_.Exception.Message -like '关节还在移动：*') {
            $status.Text = "本次点动未发出：$($_.Exception.Message)"
        } else { Show-Fault $_ }
    }
}
foreach ($row in $rows.Values) {
    $row.minus.Add_Click($jogHandler)
    $row.plus.Add_Click($jogHandler)
}
$save.Add_Click({
    if ([string]::IsNullOrWhiteSpace($poseName.Text)) {
        $status.Text = '请输入姿态名称。'
        return
    }
    $dialog = [Windows.Forms.SaveFileDialog]::new()
    $dialog.Filter = 'JSON (*.json)|*.json'
    $dialog.InitialDirectory = [Environment]::GetFolderPath('MyDocuments')
    $dialog.FileName = ($poseName.Text -replace '[^\p{L}\p{N}_-]','_') + '.json'
    try {
        if ($dialog.ShowDialog($form) -eq 'OK') {
            $joints = @(Read-Arm)
            Save-Pose $dialog.FileName $poseName.Text $joints
            $status.Text = "已保存 $($dialog.FileName)"
        }
    } catch { Show-Fault $_ } finally { $dialog.Dispose() }
})
$timer.Add_Tick({
    try {
        Show-Arm
        foreach ($id in @($script:pending.Keys)) {
            $pending = $script:pending[$id]
            if ([DateTimeOffset]::Now -lt $pending.after) { continue }
            $script:pending.Remove($id)
            Write-Diagnostic 'feedback' @{id=$id; commanded_target=$pending.target; measured=(Read-JointFeedback $id)}
        }
    } catch { $timer.Stop(); Show-Fault $_ }
})
$form.Add_FormClosing({
    param($sender,$eventArgs)
    if ($script:holding -or $script:locked) {
        $answer = [Windows.Forms.MessageBox]::Show(
            '关闭窗口不会释放力矩。请先取消，托住手臂后点「支撑后释放」。只有已切断舵机电源才继续关闭。',
            '关节可能仍有力矩', 'OKCancel', 'Warning',
            [Windows.Forms.MessageBoxDefaultButton]::Button2)
        if ($answer -ne 'OK') { $eventArgs.Cancel = $true }
    }
})
Update-Buttons
try { [void]$form.ShowDialog() } finally {
    $timer.Dispose()
    if ($script:bus) { $script:bus.Dispose() }
    $form.Dispose()
}
