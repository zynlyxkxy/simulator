import sys, os
os.environ['QT_DEBUG_PLUGINS'] = '1'
from PyQt5.QtCore import qInstallMessageHandler, QtDebugMsg, QtInfoMsg, QtWarningMsg, QtCriticalMsg, QtFatalMsg

log = open(r'C:\Users\e-hao.wu\Desktop\3000测校SOP_u\HoriMetrologyScirptsFramework\_qt_debug_out.txt', 'w', encoding='utf-8')

def handler(mode, ctx, msg):
    log.write(f"[{mode}] {msg}\n")
    log.flush()

qInstallMessageHandler(handler)

from PyQt5.QtWidgets import QApplication
app = QApplication(sys.argv)
log.write("OK: QApplication created\n")
log.close()
