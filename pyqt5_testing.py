#https://stackoverflow.com/questions/52098156/launch-javascript-function-from-pyqt-qwebengineview

import sys, base64, os
import folium # pip install folium
import jinja2
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView # pip install PyQtWebEngine
from PyQt5 import QtCore

# created a folium map inside the class MyApp
class MyApp(QWidget):
    def __init__(self):
        super().__init__()  # referencing the "parent" or "super" class, QWidget which the child class MyApp needs.
        self.setWindowTitle('Disengagment Report Map')
        self.window_width, self.window_height = 1600, 1200
        self.setMinimumSize(self.window_width, self.window_height)

        layout = QVBoxLayout()
        self.setLayout(layout)

        center = (37.377223, -121.990215)
        m = folium.Map(tiles='Stamen Terrain', zoom_start=14, location=center, control_scale=True)

        img = '20230113_154702.gif'
        encoded = base64.b64encode(open(img, 'rb').read()).decode()
        gif_html = f'<img src="data:image/gif;base64,{encoded}">'

        html = f"""{gif_html}
                    <br>DESCRIPTION OF FACTS CAUSING DISENGAGEMENT: <input type="text" value="{"The AV steered offcourse"}" id="myInput"><br> 
                    <button onclick="myFunction()">Save</button>"""
            
        folium.Marker(location=[37.391200, -122.001061], popup=html).add_to(m)

        java = folium.MacroElement().add_to(m)
        java._template = jinja2.Template("""
            {% macro script(this, kwargs) %}
            function myFunction() {
                var copyText = document.getElementById("myInput");
                copyText.select();
                //navigator.clipboard.writeText(copyText.value); //does not work with qwebengineview()
                document.execCommand('copy');
            }
            {% endmacro %}
        """)

        m.save('map.html')

        url = QtCore.QUrl.fromLocalFile(os.path.join(os.getcwd(), 'map.html'))

        webView = QWebEngineView()  # used to view and edit web documents
        webView.load(url)   
        layout.addWidget(webView)


app = QApplication(sys.argv)
app.setStyleSheet('QWidget {font-size: 35px;}')

myApp = MyApp() # defining instance
myApp.show()

app.exec()