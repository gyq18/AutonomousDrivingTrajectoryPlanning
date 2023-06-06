echo off

echo Attaching to dongle.

lmutil lmreread -c "%MOSEKLM_LICENSE_FILE%"

echo Done.
pause
