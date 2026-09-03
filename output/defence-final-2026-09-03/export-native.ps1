param([Parameter(Mandatory=$true)][string]$Deck,[Parameter(Mandatory=$true)][string]$Pdf,[Parameter(Mandatory=$true)][string]$RenderDir)
$ErrorActionPreference='Stop'
$Deck=[System.IO.Path]::GetFullPath($Deck)
$Pdf=[System.IO.Path]::GetFullPath($Pdf)
$RenderDir=[System.IO.Path]::GetFullPath($RenderDir)
$app=New-Object -ComObject PowerPoint.Application
$previousAlerts=$app.DisplayAlerts
$doc=$null
try {
  $app.DisplayAlerts=1
  $doc=$app.Presentations.Open2007($Deck,-1,0,0,0)
  if($doc.Slides.Count -ne 62 -or $doc.Saved -ne -1){throw 'Unexpected slide count or unsaved state after repair-disabled open.'}
  [void][System.IO.Directory]::CreateDirectory($RenderDir)
  $manifest=@()
  foreach($slide in $doc.Slides){
    $path=Join-Path $RenderDir ('slide-{0:D2}.png' -f $slide.SlideIndex)
    $slide.Export($path,'PNG',1280,720)
    $manifest+=[pscustomobject]@{slide=$slide.SlideIndex;shapes=$slide.Shapes.Count;path=$path}
  }
  # Native PDF export; this does not save or modify the source PowerPoint package.
  $doc.SaveAs($Pdf,32)
  [pscustomobject]@{PowerPointVersion=$app.Version;RepairRequested=$false;Slides=$manifest.Count;Pdf=$Pdf;Rendered=$manifest} | ConvertTo-Json -Depth 4
} finally {
  if($null -ne $doc){$doc.Close();[void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($doc)}
  $app.DisplayAlerts=$previousAlerts
  [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app)
}
