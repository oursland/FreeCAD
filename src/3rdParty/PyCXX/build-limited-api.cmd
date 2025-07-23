setlocal
rem M.m e.g. 2.7 3.6 etc
set PYTHON_VER=%1
rem 32 or 64
set PYTHON_ARCH=%2
rem 10.0, 14.0, 2017
set VC_VER=%3
rem 3.5 etc
set API=%4

echo ------------------------------------------------------
echo Testing limited API %4 for python %1 %2 using VC %3
echo ------------------------------------------------------
if %PYTHON_ARCH% == win32 (
    set PYTHON_VER_ARCH=%PYTHON_VER%-32

    if exist "C:\Program Files (x86)\Microsoft Visual Studio %VC_VER%\VC\vcvarsall.bat" (
        call "C:\Program Files (x86)\Microsoft Visual Studio %VC_VER%\VC\vcvarsall.bat"
    )
    if exist "c:\Program Files (x86)\Microsoft Visual Studio\%VC_VER%\Community\VC\Auxiliary\Build\vcvars32.bat" (
        call "c:\Program Files (x86)\Microsoft Visual Studio\%VC_VER%\Community\VC\Auxiliary\Build\vcvars32.bat"
    )
)
if %PYTHON_ARCH% == win64 (
    set PYTHON_VER_ARCH=%PYTHON_VER%-64

    if exist "C:\Program Files (x86)\Microsoft Visual Studio %VC_VER%\VC\bin\amd64\vcvars64.bat" (
        call "C:\Program Files (x86)\Microsoft Visual Studio %VC_VER%\VC\bin\amd64\vcvars64.bat"
    )
    if exist "c:\Program Files (x86)\Microsoft Visual Studio\%VC_VER%\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "c:\Program Files (x86)\Microsoft Visual Studio\%VC_VER%\Community\VC\Auxiliary\Build\vcvars64.bat"
    )
)

py -%PYTHON_VER_ARCH% setup_makefile.py %PYTHON_ARCH% tmp-%PYTHON_ARCH%-python%PYTHON_VER%-limited-%API%-build.mak --limited-api=%API%
if errorlevel 1 exit /b 1
nmake -f tmp-%PYTHON_ARCH%-python%PYTHON_VER%-limited-%API%-build.mak clean all 2>&1 | py -3 build_tee.py tmp-%PYTHON_ARCH%-python%PYTHON_VER%-limited-%API%-build.log
if not exist obj\pycxx_iter.pyd exit /b 1
nmake -f tmp-%PYTHON_ARCH%-python%PYTHON_VER%-limited-%API%-build.mak test 2>&1 | py -3 build_tee.py -a tmp-%PYTHON_ARCH%-python%PYTHON_VER%-limited-%API%-test.log
echo All done
endlocal
