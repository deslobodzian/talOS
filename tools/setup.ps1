#
# talos developer environment on Windows. talOS needs POSIX shm and signals,
# so the Bazel tree builds only inside WSL2: this ensures WSL2 + a distro and
# runs tools/setup.sh there, then installs the native tools for the RoboRIO
# Gradle project, Studio and WPILib. Development only; deployment is
# tools/deploy.sh.
#
#   powershell -ExecutionPolicy Bypass -File tools\setup.ps1 [options]
#
#   -Distro NAME   WSL distro (default Ubuntu-24.04)
#   -Check         report only, install nothing (still runs setup.sh --check)
#   -Yes           no prompts
#   -NoStudio      skip Node/pnpm (WSL and native)
#   -NoRio         skip JDK 17 / RoboRIO toolchain (WSL and native)
#   -NoLint        skip clang-format/clang-tidy/buildifier in WSL
#   -Verify        build the launcher and run --describe-only in WSL
#   -NoNative      skip the native Windows tools (winget)
#
# Exit: 0 ok; 1 something required is missing or failed (or setup.sh's own
# non-zero code); 2 WSL needs a reboot or first launch, then rerun.
#
[CmdletBinding()]
param(
  [string]$Distro = 'Ubuntu-24.04',
  [switch]$Check,
  [switch]$Yes,
  [switch]$NoStudio,
  [switch]$NoRio,
  [switch]$NoLint,
  [switch]$Verify,
  [switch]$NoNative
)
$ErrorActionPreference = 'Stop'

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).ProviderPath
$PnpmVersion = '9.15.9'
$WpilibUrl = 'https://github.com/wpilibsuite/allwpilib/releases'
# wsl.exe prints UTF-16 unless told otherwise; NULs are stripped either way.
$env:WSL_UTF8 = '1'
# A corepack pnpm shim must never stop at an interactive download prompt.
$env:COREPACK_ENABLE_DOWNLOAD_PROMPT = '0'
# -Check installs nothing, so a shim must not fetch pnpm either.
if ($Check) { $env:COREPACK_ENABLE_NETWORK = '0' }

$Rows = New-Object System.Collections.ArrayList
$script:Failed = $false
$script:StopMsg = $null
$script:ShCode = 0
$script:WslRoot = $null

function Say($m) { Write-Host "setup: $m" }
function Warn($m) { Write-Host "setup: warning: $m" -ForegroundColor Yellow }
function Section($m) { Write-Host ''; Write-Host "--- $m ---" -ForegroundColor Cyan }
function Has($cmd) { [bool](Get-Command $cmd -ErrorAction SilentlyContinue) }

function Row($name, $status, $detail) {
  [void]$Rows.Add([pscustomobject]@{ Component = $name; Status = $status; Detail = "$detail" })
  if ('missing', 'wrong', 'failed' -contains $status) { $script:Failed = $true }
}

# Runs a native command and captures stdout+stderr as text. Stderr must not
# reach PowerShell's error stream: under 'Stop' that throws on PS 5.1.
function Exec([string]$Exe, [string[]]$ArgList = @()) {
  $eap = $ErrorActionPreference
  $ErrorActionPreference = 'Continue'
  try {
    $out = @(& $Exe @ArgList 2>&1 | ForEach-Object { "$_" -replace "`0", '' })
    $code = $LASTEXITCODE
  } catch {
    $out = @("$_")
    $code = -1
  } finally {
    $ErrorActionPreference = $eap
  }
  [pscustomobject]@{
    Code = $code
    Out  = @($out | Where-Object { $_.Trim() -ne '' })
    Text = ($out -join "`n")
  }
}

# Runs a native command attached to this console (sudo prompts, progress
# bars and colors need a real terminal) and returns its exit code.
function Run([string]$Exe, [string[]]$ArgList = @()) {
  # CommandLineToArgvW quoting; Start-Process on PS 5.1 takes one string.
  $line = ($ArgList | ForEach-Object {
    if ($_ -ne '' -and $_ -notmatch '[\s"]') { $_ }
    else { '"' + (($_ -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"' }
  }) -join ' '
  $exePath = (Get-Command $Exe).Source
  $p = Start-Process -FilePath $exePath -ArgumentList $line -NoNewWindow -PassThru
  $null = $p.Handle  # PS 5.1: ExitCode stays empty unless the handle is opened early
  $p.WaitForExit()
  return $p.ExitCode
}

function Confirm-Step($q) {
  if ($Yes) { return $true }
  $a = Read-Host "setup: $q [Y/n]"
  return ($a -eq '' -or $a -match '^[yY]')
}

function Test-Admin {
  $id = [Security.Principal.WindowsIdentity]::GetCurrent()
  (New-Object Security.Principal.WindowsPrincipal $id).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)
}

# winget installs land in the registry PATH, not in this session's.
function Update-Path {
  $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
              [Environment]::GetEnvironmentVariable('Path', 'User')
}

# --- WSL ---------------------------------------------------------------------
# name -> WSL version, from `wsl -l -v` lines like "* Ubuntu-24.04  Stopped  2".
function Get-WslDistros {
  $map = @{}
  $r = Exec 'wsl.exe' @('--list', '--verbose')
  if ($r.Code -ne 0) { return $map }
  foreach ($l in $r.Out) {
    if ($l -match '^\s*\*?\s*(\S+)\s+\S+\s+([12])\s*$') { $map[$Matches[1]] = [int]$Matches[2] }
  }
  return $map
}

# Returns $true when $Distro is a usable WSL2 distro.
function Initialize-Wsl {
  if (-not (Has 'wsl.exe') -or (Exec 'wsl.exe' @('--status')).Code -ne 0) {
    if ($Check) { Row 'WSL' 'missing' 'rerun without -Check from an elevated PowerShell'; return $false }
    if (-not (Test-Admin)) {
      Row 'WSL' 'missing' 'installing WSL needs an elevated PowerShell (Run as administrator); rerun from one'
      return $false
    }
    if (-not (Confirm-Step "install WSL2 and $Distro?")) { Row 'WSL' 'missing' 'declined'; return $false }
    $code = Run 'wsl.exe' @('--install', '-d', $Distro)
    if ($code -ne 0) { Row 'WSL' 'failed' "wsl --install -d $Distro exited $code"; return $false }
    Row 'WSL' 'installed' 'reboot required'
    $script:StopMsg = "reboot Windows, let $Distro finish its first launch (create your Linux user), then rerun tools\setup.ps1"
    return $false
  }
  $v = Exec 'wsl.exe' @('--version')
  Row 'WSL' 'ok' $(if ($v.Code -eq 0 -and $v.Out) { $v.Out[0].Trim() } else { '' })

  $distros = Get-WslDistros
  if (-not $distros.ContainsKey($Distro)) {
    if ($Check) { Row $Distro 'missing' "wsl --install -d $Distro"; return $false }
    if (-not (Confirm-Step "install WSL distro $Distro?")) { Row $Distro 'missing' 'declined'; return $false }
    $code = Run 'wsl.exe' @('--install', '-d', $Distro, '--no-launch')
    if ($code -ne 0) { Row $Distro 'failed' "wsl --install -d $Distro exited $code"; return $false }
    Row $Distro 'installed' 'first launch pending'
    $script:StopMsg = "run 'wsl -d $Distro' once to create your Linux user, exit it, then rerun tools\setup.ps1"
    return $false
  }

  if ($distros[$Distro] -ne 2) {
    if ($Check) { Row $Distro 'wrong' "WSL1; fix: wsl --set-version $Distro 2"; return $false }
    Say "converting $Distro to WSL2"
    if ((Run 'wsl.exe' @('--set-version', $Distro, '2')) -ne 0) {
      Row $Distro 'failed' 'wsl --set-version failed; enable Virtual Machine Platform (elevated: wsl --install --no-distribution), reboot, rerun'
      return $false
    }
  }

  $u = Exec 'wsl.exe' @('-d', $Distro, '-e', 'id', '-un')
  if ($u.Code -ne 0 -or -not $u.Out) { Row $Distro 'failed' ("cannot start: " + ($u.Out -join ' ')); return $false }
  $user = $u.Out[-1].Trim()
  # Per-user installs (corepack, ~/.gradle, ~/.cache/bazel) would land in /root.
  if ($user -eq 'root') {
    Row $Distro 'wrong' 'WSL2, but the default user is root'
    $script:StopMsg = ("$Distro has no regular default user. Run 'wsl -d $Distro' to finish its first launch, or: " +
      "wsl -d $Distro -u root adduser <name>; wsl -d $Distro -u root usermod -aG sudo <name>; " +
      "make it the default (<distro>.exe config --default-user <name>, or [user] default=<name> in /etc/wsl.conf " +
      "then wsl --terminate $Distro); then rerun tools\setup.ps1")
    return $false
  } else {
    Row $Distro 'ok' "WSL2, user $user"
  }
  return $true
}

# --- repo path ---------------------------------------------------------------
function Get-WslRepoPath {
  # Repo opened from inside a distro (\\wsl.localhost\<distro>\...).
  if ($RepoRoot -match '^\\\\wsl(\$|\.localhost)\\([^\\]+)\\(.*)$') {
    if ($Matches[2] -ne $Distro) {
      Row 'repo path' 'failed' "repo lives in WSL distro $($Matches[2]); rerun with -Distro $($Matches[2])"
      return $null
    }
    return '/' + ($Matches[3] -replace '\\', '/')
  }
  $r = Exec 'wsl.exe' @('-d', $Distro, '-e', 'wslpath', '-a', ($RepoRoot -replace '\\', '/'))
  if ($r.Code -ne 0 -or -not $r.Out) {
    Row 'repo path' 'failed' ("wslpath: " + ($r.Out -join ' '))
    return $null
  }
  return $r.Out[-1].Trim()
}

# --- line endings ------------------------------------------------------------
# Checkouts made with core.autocrlf=true before .gitattributes pinned *.sh and
# gradlew to LF still have CRLF scripts, which bash rejects ($'\r').
function Get-CrlfScripts {
  $files = @()
  if (Has 'git') {
    $r = Exec 'git' @('-C', $RepoRoot, 'ls-files', '--', '*.sh', '*gradlew')
    if ($r.Code -eq 0) { $files = @($r.Out) }
  }
  if ($files -notcontains 'tools/setup.sh') { $files += 'tools/setup.sh' }
  @($files | Where-Object {
    $p = Join-Path $RepoRoot $_
    (Test-Path -LiteralPath $p) -and [IO.File]::ReadAllText($p).Contains("`r`n")
  })
}

# Returns $false when tools/setup.sh still can't run under bash.
function Repair-LineEndings {
  $bad = Get-CrlfScripts
  if (-not $bad) { Row 'script line endings' 'ok' 'LF'; return $true }
  $list = $bad -join ', '
  if ($Check -or -not (Confirm-Step "convert CRLF to LF in: $list?")) {
    Row 'script line endings' 'wrong' "CRLF in $list (rerun without -Check to convert)"
  } else {
    # In place, so local edits survive; with .gitattributes (eol=lf) git sees no change.
    $utf8 = New-Object System.Text.UTF8Encoding $false
    foreach ($f in $bad) {
      $p = Join-Path $RepoRoot $f
      [IO.File]::WriteAllText($p, [IO.File]::ReadAllText($p).Replace("`r`n", "`n"), $utf8)
    }
    Row 'script line endings' 'fixed' "converted to LF: $list"
    $bad = @()
  }
  return ($bad -notcontains 'tools/setup.sh')
}

# --- setup.sh in WSL ---------------------------------------------------------
function Invoke-SetupSh {
  if (-not (Test-Path -LiteralPath (Join-Path $RepoRoot 'tools\setup.sh'))) {
    Row 'tools/setup.sh' 'missing' 'not found in the checkout'
    return
  }
  Section "tools/setup.sh in $Distro"
  $flags = @()
  if ($Check) { $flags += '--check' }
  if ($Yes) { $flags += '--yes' }
  if ($NoStudio) { $flags += '--no-studio' }
  if ($NoRio) { $flags += '--no-rio' }
  if ($NoLint) { $flags += '--no-lint' }
  if ($Verify) { $flags += '--verify' }
  Say "running in ${Distro}: bash tools/setup.sh $($flags -join ' ')"
  $script:ShCode = Run 'wsl.exe' (@('-d', $Distro, '--cd', $script:WslRoot, '-e', 'bash', 'tools/setup.sh') + $flags)
  if ($script:ShCode -eq 0) {
    Row "setup.sh in $Distro" 'ok' $(if ($Check) { 'all required components present' } else { 'done' })
  } elseif ($Check -and $script:ShCode -eq 1) {
    Row "setup.sh in $Distro" 'missing' 'components missing in WSL; see its table above'
  } else {
    Row "setup.sh in $Distro" 'failed' "exit $($script:ShCode); see its output above"
  }
}

# --- native Windows tools ----------------------------------------------------
$WingetNotFound = -1978335212         # 0x8A150014: no package matches the id
$WingetAlreadyInstalled = @(-1978335189, -1978335135)  # update n/a, already installed

function Test-WingetPackage($id) {
  # Detection only: no source agreements here, so -Check writes nothing.
  (Exec 'winget' @('list', '--id', $id, '-e', '--disable-interactivity')).Code -eq 0
}

# Installs the first id winget knows. $Ids beyond the first are fallbacks used
# only when an id is unknown to the winget source.
function Install-Package($name, [string[]]$Ids, $hint) {
  $hasWinget = Has 'winget'
  if ($hasWinget) {
    foreach ($id in $Ids) {
      if (Test-WingetPackage $id) {
        Row $name 'ok' "$id installed; open a new shell if it isn't on PATH yet"
        return
      }
    }
  }
  if ($Check) { Row $name 'missing' ("winget install --id $($Ids[0]) -e" + $hint); return }
  if (-not $hasWinget) { Row $name 'missing' 'winget not found: install "App Installer" from the Microsoft Store'; return }
  if (-not (Confirm-Step "install $name (winget $($Ids[0]))?")) { Row $name 'missing' 'declined'; return }
  foreach ($id in $Ids) {
    Say "winget install $id"
    $code = Run 'winget' @('install', '--id', $id, '-e', '--accept-source-agreements', '--accept-package-agreements')
    if ($code -eq $WingetNotFound) { Warn "winget has no package $id"; continue }
    Update-Path
    if ($code -eq 0 -or $WingetAlreadyInstalled -contains $code) { Row $name 'installed' $id }
    else { Row $name 'failed' "winget install $id exited $code" }
    return
  }
  Row $name 'failed' ("no winget package found: " + ($Ids -join ', '))
}

function Get-JavaMajor($exe) {
  $r = Exec $exe @('-version')
  if ($r.Text -match 'version "(\d+)') { return [int]$Matches[1] }
  return 0
}

function Get-PnpmVersion {
  if (-not (Has 'pnpm')) { return $null }
  Push-Location $RepoRoot
  try { $r = Exec 'pnpm' @('--version') } finally { Pop-Location }
  if ($r.Code -eq 0 -and $r.Out) { return $r.Out[-1].Trim() }
  return $null
}

function Initialize-Native {
  if (Has 'git') { Row 'git (Windows)' 'ok' (Exec 'git' @('--version')).Out[0] }
  else { Install-Package 'git (Windows)' @('Git.Git') '' }

  if (-not $NoRio) {
    # gradlew uses JAVA_HOME when set, else java on PATH; GradleRIO 2026 wants 17.
    $jhMajor = 0
    if ($env:JAVA_HOME) {
      $jh = Join-Path $env:JAVA_HOME 'bin\java.exe'
      if (Test-Path -LiteralPath $jh) { $jhMajor = Get-JavaMajor $jh }
    }
    $java = Get-Command java -ErrorAction SilentlyContinue
    $pathMajor = if ($java) { Get-JavaMajor $java.Source } else { 0 }
    if ($jhMajor -eq 17) {
      Row 'JDK 17 (Windows)' 'ok' "JAVA_HOME=$env:JAVA_HOME"
    } elseif (-not $env:JAVA_HOME -and $pathMajor -eq 17) {
      Row 'JDK 17 (Windows)' 'ok' $java.Source
    } else {
      $wpiJdk = Join-Path $env:PUBLIC 'wpilib\2026\jdk'
      $hint = if (Test-Path -LiteralPath $wpiJdk) { " (or set JAVA_HOME=$wpiJdk)" } else { '' }
      Install-Package 'JDK 17 (Windows)' @('EclipseAdoptium.Temurin.17.JDK') $hint
      if ($env:JAVA_HOME -and $jhMajor -ne 17) {
        Row 'JAVA_HOME' 'wrong' "points to Java $jhMajor ($env:JAVA_HOME); gradlew uses it, point it at a JDK 17"
      }
    }
  }

  if (-not $NoStudio) {
    $nodeMajor = 0
    if (Has 'node') {
      $nv = (Exec 'node' @('--version')).Out
      if ($nv -and $nv[0] -match '^v(\d+)') { $nodeMajor = [int]$Matches[1] }
    }
    if ($nodeMajor -ge 22) {
      Row 'Node.js (Windows)' 'ok' "$($nv[0]) (Bazel pins 22.14.0)"
    } else {
      Install-Package 'Node.js (Windows)' @('OpenJS.NodeJS.22', 'OpenJS.NodeJS.LTS') ''
    }

    $pv = Get-PnpmVersion
    if ($pv -eq $PnpmVersion) {
      Row 'pnpm (Windows)' 'ok' $pv
    } elseif ($Check) {
      $got = if ($pv) { "found $pv" } else { 'not found' }
      Row 'pnpm (Windows)' $(if ($pv) { 'wrong' } else { 'missing' }) "$got; corepack enable; corepack prepare pnpm@$PnpmVersion --activate"
    } elseif (-not (Has 'node')) {
      Row 'pnpm (Windows)' 'missing' 'needs Node.js on PATH; open a new shell and rerun'
    } else {
      # Node >= 25 no longer bundles corepack.
      if (-not (Has 'corepack')) { [void](Exec 'npm' @('install', '-g', 'corepack')); Update-Path }
      # Plain `corepack enable` writes shims next to node.exe (Program Files
      # needs admin); fall back to npm's per-user bin dir, which is on PATH.
      $r = Exec 'corepack' @('enable')
      if ($r.Code -ne 0) {
        $dir = Join-Path $env:APPDATA 'npm'
        New-Item -ItemType Directory -Force -Path $dir | Out-Null
        $r = Exec 'corepack' @('enable', '--install-directory', $dir)
        if (($env:Path -split ';') -notcontains $dir) { Warn "$dir is not on PATH; add it for pnpm" ; $env:Path += ";$dir" }
      }
      $r = Exec 'corepack' @('prepare', "pnpm@$PnpmVersion", '--activate')
      if ($r.Code -ne 0) { $r = Exec 'corepack' @('install', '-g', "pnpm@$PnpmVersion") }
      $pv = Get-PnpmVersion
      if ($pv -eq $PnpmVersion) { Row 'pnpm (Windows)' 'installed' "$pv via corepack" }
      else { Row 'pnpm (Windows)' 'failed' ("corepack: " + ($r.Out -join ' ')) }
    }
  }

  $wpi = Join-Path $env:PUBLIC 'wpilib\2026'
  if (Test-Path -LiteralPath $wpi) { Row 'WPILib 2026' 'ok' $wpi }
  else { Row 'WPILib 2026' 'manual' "installer (not on winget): $WpilibUrl" }
  $ds = Join-Path ${env:ProgramFiles(x86)} 'FRC Driver Station'
  if (Test-Path -LiteralPath $ds) { Row 'Driver Station' 'ok' $ds }
  else { Row 'Driver Station' 'manual' 'NI FRC Game Tools (only needed to drive a real robot)' }
}

# --- main --------------------------------------------------------------------
Section 'WSL'
if (Initialize-Wsl) {
  $script:WslRoot = Get-WslRepoPath
  if ($script:WslRoot) {
    if ($script:WslRoot -like '/mnt/*') {
      Row 'repo location' 'warn' ("$($script:WslRoot) is on the Windows drive: Bazel I/O there is slow. " +
        "Prefer cloning inside WSL (cd ~ && git clone <url> talos); .gitattributes keeps *.sh LF either way")
    } else {
      Row 'repo location' 'ok' $script:WslRoot
    }
    if (Repair-LineEndings) {
      Invoke-SetupSh
    } else {
      Row "setup.sh in $Distro" 'skipped' 'tools/setup.sh has CRLF line endings'
    }
  }
}

if (-not $NoNative) {
  Section 'native Windows tools'
  Initialize-Native
}

# --- summary -----------------------------------------------------------------
Section 'summary'
$w = ($Rows | ForEach-Object { $_.Component.Length } | Measure-Object -Maximum).Maximum
foreach ($r in $Rows) {
  $color = switch ($r.Status) {
    { 'ok', 'installed', 'fixed' -contains $_ } { 'Green'; break }
    { 'missing', 'wrong', 'failed' -contains $_ } { 'Red'; break }
    { 'warn', 'manual', 'skipped' -contains $_ } { 'Yellow'; break }
    default { 'Gray' }
  }
  Write-Host ('  {0}  {1}  {2}' -f $r.Component.PadRight($w), $r.Status.PadRight(9), $r.Detail) -ForegroundColor $color
}

Write-Host ''
if ($script:StopMsg) {
  Say "next: $($script:StopMsg)"
  exit 2
}
$cdArg = if ($script:WslRoot) { " --cd $($script:WslRoot)" } else { '' }
Write-Host @"
Next steps (Bazel tree, in WSL):
  wsl -d $Distro$cdArg
  bazel build //...
  ./sim.sh --sim-gateway --no-studio   # robot stack with the lightweight simulator
  ./sim.sh                             # WPILib sim (GUI via WSLg)

Native Windows:
  cd 2026-robot\controller_processor\rio; .\gradlew.bat installRoboRioToolchain
  cd studio; pnpm install; pnpm test
  WPILib 2026 (VS Code, sim GUI tools): $WpilibUrl
  Driver Station: NI FRC Game Tools

A native-Windows WPILib sim talking UDP to nodes in WSL needs mirrored
networking: add to %UserProfile%\.wslconfig
  [wsl2]
  networkingMode=mirrored
then run 'wsl --shutdown'. This script does not edit .wslconfig.
"@

if ($script:ShCode -ne 0) { exit $script:ShCode }
if ($script:Failed) { exit 1 }
exit 0
