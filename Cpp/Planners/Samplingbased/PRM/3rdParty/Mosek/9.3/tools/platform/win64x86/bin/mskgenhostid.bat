echo off

echo Hostname: > "%USERPROFILE%\hostid.txt"
hostname >> "%USERPROFILE%\hostid.txt"
echo HOST ID: >> "%USERPROFILE%\hostid.txt"
lmutil lmhostid >> "%USERPROFILE%\hostid.txt"
echo FLEX ID: >> "%USERPROFILE%\hostid.txt"
lmutil lmhostid -flexid >> "%USERPROFILE%\hostid.txt"

echo --------------------------------------------------------------------
echo Hostname:
hostname
echo --------------------------------------------------------------------
echo HOST ID from network card MAC address:
lmutil lmhostid
echo --------------------------------------------------------------------
echo HOSTID from dongle (if present):
lmutil lmhostid -flexid
echo --------------------------------------------------------------------

echo ********************************************************************
echo * ONLY FOR COMMERCIAL LICENSES !!                                  
echo * Academic and trial licenses: see our website, no HOSTID needed.  
echo ********************************************************************
echo * The above output shows your HOSTNAME and HOSTID. 
echo * MOSEK has also written a file containing the HOSTID and HOSTNAME to:
echo * "%USERPROFILE%\hostid.txt"         
echo * Please send this file to license@mosek.com to receive your license.
echo ********************************************************************
pause                     
