#!/usr/bin/env python3

from PyQt5 import QtCore, QtWidgets

from linear_move_ui import LinearMoveWindow

from joint_move_ui import JointMoveWindow

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("main_window")
        self.setFixedSize(480, 640)


        self.bg_path = "TODO"

        self.btn_width = 150
        self.btn_height = 20

        self.btns = [
            "linear_move",
            "joint_move",
            "temp01",
            "temp02",
            "temp03",
            "temp04"
        ]
        # 回调函数（顺序必须和 btns 对应）
        self.btns_cb = [
            self.linear_move_cb,
            self.joint_move_cb,
            self.test01_cb,
            self.test02_cb,
            self.test03_cb,
            self.test04_cb
        ]
        # 存放子窗口，closeEvent 时关闭
        self.child_windows = []
        
        self._build_ui()



    def _build_ui(self):
        # 设置背景图片

        # 创建居中标题
        title = QtWidgets.QLabel("main_window")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")

        # 创建网格布局
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(15)

        # 第一列从上往下创建4个标签，第2列从上往下创建2个标签，下面2个留白
        for idx,btn in enumerate(self.btns):
            button = QtWidgets.QPushButton(btn)
            self.set_button_size(button,self.btn_width,self.btn_height)
            row = idx%4
            col = idx//4
            grid.addWidget(button,row,col)
            # 点击后就会进入不同的子窗口，也就是触发不同的回调函数
            button.clicked.connect(self.btns_cb[idx])

        # 整体纵向布局
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addLayout(grid)
        layout.addStretch(1)
        self.setLayout(layout)


    def linear_move_cb(self):
        win = LinearMoveWindow()
        win.show()
        self.child_windows.append(win)

    def joint_move_cb(self):
        win = JointMoveWindow()
        win.show()
        self.child_windows.append(win)

    def test01_cb(self):
        win = SubWindow("Temp 01")
        win.show()
        self.child_windows.append(win)

    def test02_cb(self):
        win = SubWindow("Temp 02")
        win.show()
        self.child_windows.append(win)

    def test03_cb(self):
        win = SubWindow("Temp 03")
        win.show()
        self.child_windows.append(win)

    def test04_cb(self):
        win = SubWindow("Temp 04")
        win.show()
        self.child_windows.append(win)

    def set_button_size(self, button, width, height):
        button.setFixedSize(width, height)


    def closeEvent(self, event):
        # 关闭所有子窗口
        for w in self.child_windows:
            if w.isVisible():
                w.close()
        event.accept()


class SubWindow(QtWidgets.QWidget):
    def __init__(self, title="SubWindow"):
        super().__init__()
        self.setWindowTitle(title)
        self.setFixedSize(400, 300)

        layout = QtWidgets.QVBoxLayout()
        label = QtWidgets.QLabel(title)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 20px;")

        layout.addWidget(label)
        layout.addStretch(1)
        self.setLayout(layout)


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())
