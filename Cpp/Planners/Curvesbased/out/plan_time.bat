@echo off
set n=0
:abc
../build/PathPlanning.exe
set /a n+=1
if %n%==99 exit
goto abc