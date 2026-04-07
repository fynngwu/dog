MAIN_STYLE = """
QMainWindow {
    background-color: #f5f6fa;
}
QWidget {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    font-size: 13px;
    color: #2f3640;
}
QPushButton {
    background-color: #0097e6;
    color: white;
    border: none;
    border-radius: 6px;
    padding: 7px 14px;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #00a8ff;
}
QPushButton:disabled {
    background-color: #dcdde1;
    color: #7f8fa6;
}
QPushButton#danger {
    background-color: #e84118;
}
QPushButton#danger:hover {
    background-color: #c23616;
}
QLineEdit, QSpinBox, QDoubleSpinBox, QTextEdit, QListWidget {
    border: 1px solid #dcdde1;
    border-radius: 6px;
    padding: 4px;
    background-color: white;
}
QGroupBox {
    border: 1px solid #dcdde1;
    border-radius: 8px;
    margin-top: 10px;
    background-color: white;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    color: #353b48;
    font-weight: bold;
}
QTabWidget::pane {
    border: 1px solid #dcdde1;
    background: white;
    border-radius: 4px;
}
QTabBar::tab {
    background: #f5f6fa;
    border: 1px solid #dcdde1;
    padding: 8px 16px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}
QTabBar::tab:selected {
    background: white;
    border-bottom-color: white;
    font-weight: bold;
    color: #0097e6;
}
"""
