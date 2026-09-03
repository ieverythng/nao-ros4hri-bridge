param([Parameter(Mandatory=$true)][string]$Deck)
$ErrorActionPreference='Stop'
Add-Type -AssemblyName WindowsBase
Add-Type -Path 'C:/Program Files/Microsoft Office/root/vfs/ProgramFilesX86/Microsoft Office/Office16/DCF/DocumentFormat.OpenXml.dll'
$doc=[DocumentFormat.OpenXml.Packaging.PresentationDocument]::Open($Deck,$false)
try {
  $validator=New-Object DocumentFormat.OpenXml.Validation.OpenXmlValidator
  $errors=@($validator.Validate($doc))
  $errors | ForEach-Object {
    [pscustomobject]@{Description=$_.Description;Part=$_.Part.Uri.ToString();Path=$_.Path.XPath;Type=$_.ErrorType.ToString();Id=$_.Id}
  } | ConvertTo-Json -Depth 6
  Write-Output "VALIDATION_ERRORS=$($errors.Count)"
  if($errors.Count -gt 0){exit 1}
} finally {$doc.Dispose()}
