echo off
echo *************************************************************************************
echo * A basic test of MOSEK 
echo * Attempting to solve a linear problem
echo *************************************************************************************
mkdir "%TEMP%\mosektest"
mosek -f -d MSK_IPAR_LICENSE_DEBUG MSK_ON -baso "%TEMP%\mosektest\out.bas" -itro "%TEMP%\mosektest\out.sol" ..\..\..\..\tools\examples\data\25fv47.mps
echo ************************************************************************************* 
echo * If the above linear problem solved
echo * then MOSEK command line tool works correctly
echo * and was able to check out a license.
echo *************************************************************************************
echo * If there were problems with running MOSEK, 
echo * find the file "%USERPROFILE%\moseklog.txt" that was just created
echo * and send with your support question to support@mosek.com. 
echo *************************************************************************************
echo ************************************************************************************* > "%USERPROFILE%\moseklog.txt"
mosek -f -d MSK_IPAR_LICENSE_DEBUG MSK_ON -baso "%TEMP%\mosektest\out.bas" -itro "%TEMP%\mosektest\out.sol" ..\..\..\..\tools\examples\data\25fv47.mps >> "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
echo Windows architecture: >>  "%USERPROFILE%\moseklog.txt"
echo %PROCESSOR_ARCHITECTURE% >>  "%USERPROFILE%\moseklog.txt"
echo ************************************************************************************* >> "%USERPROFILE%\moseklog.txt"
rmdir /s /q "%TEMP%\mosektest"
pause
