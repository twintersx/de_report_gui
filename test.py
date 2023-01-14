import sys
import io
import base64
import folium # pip install folium
from folium import IFrame
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView # pip install PyQtWebEngine
from PyQt5 import QtCore
import time

"""
Folium in PyQt5
"""
class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Disengagment Map')
        self.window_width, self.window_height = 1600, 1200
        self.setMinimumSize(self.window_width, self.window_height)

        layout = QVBoxLayout()
        self.setLayout(layout)

        center = [37.377223, -121.990215]
        map_AILSV = folium.Map(tiles='Stamen Terrain', zoom_start=13, location=center, control_scale=True)

        test_image = '20230113_154702.gif'
        encoded = base64.b64encode(open(test_image, 'rb').read()).decode()

        html = '<img src="data:image/gif;base64,{}">'.format
        iframe = IFrame(html(encoded), width=600, height=600)
        popup = folium.Popup(iframe, max_width=600)

        folium.vector_layers.Marker(location=[37.391200, -122.001061], tooltip='DE #5', popup = popup).add_to(map_AILSV)

        url =r"C:\Users\X076979\Documents\Projects\de_report_gui\map.html"
        map_AILSV.save(url)

        webView = QWebEngineView()
        html_map = QtCore.QUrl.fromLocalFile(url)
        webView.load(html_map)
        layout.addWidget(webView)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet('''QWidget {font-size: 35px;}''')
    
    myApp = MyApp()
    myApp.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')
