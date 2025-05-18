@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\ubc\ELEC 291\project2\New folder\PrintADC_1\"
"C:\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\ubc\ELEC 291\project2\New folder\PrintADC_1\main.c"
if not exist hex2mif.exe goto done
if exist main.ihx hex2mif main.ihx
if exist main.hex hex2mif main.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\ubc\ELEC 291\project2\New folder\PrintADC_1\main.hex
