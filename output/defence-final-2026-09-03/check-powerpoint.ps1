param([Parameter(Mandatory=$true)][string]$Deck)
$ErrorActionPreference='Stop'
$app=New-Object -ComObject PowerPoint.Application
$previousAlerts=$app.DisplayAlerts
$doc=$null
try {
  $app.DisplayAlerts=1
  Write-Output 'Opening read-only, without a window, OpenAndRepair=false.'
  $doc=$app.Presentations.Open2007($Deck,-1,0,0,0)
  [pscustomobject]@{Opened=$true;Slides=$doc.Slides.Count;Saved=$doc.Saved;ReadOnly=$doc.ReadOnly;PowerPointVersion=$app.Version;RepairRequested=$false} | ConvertTo-Json
} catch {
  Write-Output $_.Exception.ToString()
  exit 1
} finally {
  if($null -ne $doc){$doc.Close();[void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc)}
  $app.DisplayAlerts=$previousAlerts
  [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app)
}
