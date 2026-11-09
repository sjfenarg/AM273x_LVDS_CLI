@echo off
REM ===========================================================================
REM  Headless build for AM273x_LVDS_CLI (MSS R5F) with the vendored, patched
REM  TI mmWave control library (am273x\mmwave_ctrl).
REM
REM  The TI makefiles expect this project at
REM    <SDK>\ti\utils\test\cascade\am273x_LVDS_CLI
REM  so a directory junction pointing back at this repository is created there
REM  (no SDK files are modified or copied).
REM
REM  Usage:  build.bat            (incremental)
REM          build.bat clean      (clean objects and output)
REM ===========================================================================
setlocal

set MMWAVE_SDK_DEVICE=am273x
set DOWNLOAD_FROM_CCS=yes
set MMWAVE_SDK_TOOLS_INSTALL_PATH=C:/ti/mmwave_mcuplus_sdk_04_06_01_02
set MMWAVE_SDK_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mmwave_mcuplus_sdk_04_06_01_02
set CCS_INSTALL_PATH=C:/ti/ccs1270
set R5F_CLANG_INSTALL_PATH=%CCS_INSTALL_PATH%/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS
set XDC_INSTALL_PATH=%CCS_INSTALL_PATH%/xdctools_3_62_01_16_core
set MMWAVE_XWR2XXX_DFP_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mmwave_dfp_02_02_04_00
set MCU_PLUS_AM273X_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mcu_plus_sdk_am273x_09_02_00_52
set MCU_PLUS_AWR294X_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mcu_plus_sdk_awr294x_09_02_00_15
set MCU_PLUS_AWR2544_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mcu_plus_sdk_awr2544_09_01_00_05
set SYSCONFIG_INSTALL_PATH=C:/ti/sysconfig_1.22.0
set MMWAVE_AWR294X_DFP_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mmwave_dfp_02_04_14_00
set C66X_CODEGEN_INSTALL_PATH=%CCS_INSTALL_PATH%/ccs/tools/compiler/ti-cgt-c6000_8.3.12
set C66x_DSPLIB_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/dsplib_c66x_3_4_0_0
set C66x_MATHLIB_INSTALL_PATH=%MMWAVE_SDK_TOOLS_INSTALL_PATH%/mathlib_c66x_3_1_2_1
set PATH=%CCS_INSTALL_PATH%\ccs\utils\bin;%CCS_INSTALL_PATH%\ccs\utils\cygwin;%PATH%

set "LINKDIR=C:\ti\mmwave_mcuplus_sdk_04_06_01_02\mmwave_mcuplus_sdk_04_06_01_02\ti\utils\test\cascade\am273x_LVDS_CLI"
if not exist "%LINKDIR%\makefile" (
    echo Creating junction %LINKDIR% -^> %~dp0
    mklink /J "%LINKDIR%" "%~dp0." || exit /b 1
)

pushd "%LINKDIR%"
if /I "%1"=="clean" (
    gmake mssTestClean
) else (
    gmake mssTest
)
set RC=%ERRORLEVEL%
popd
exit /b %RC%
