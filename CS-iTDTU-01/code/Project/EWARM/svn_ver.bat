cd /D %~dp0
echo #pragma once >> %1\SVNVersion.h
del SVNVersion.h \q

set /p="#define SVN_VERSION " > SVNVersion.h<nul
svnversion >> SVNVersion.h
exit
::pause