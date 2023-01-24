import sys, os, jinja2, js2py
import folium # pip install folium
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineProfile # pip install PyQtWebEngine
from PyQt5.QtCore import * 

class DeGUI(QWidget):
    layout = QVBoxLayout()

    def __init__(self):
        super().__init__()
        self.webView = QWebEngineView() 
        self.profile = QWebEngineProfile().defaultProfile()
        self.profile.setPersistentCookiesPolicy(self.profile.ForcePersistentCookies)
        print(self.profile.persistentCookiesPolicy())
        self.profile.setPersistentStoragePath(os.path.join(os.getcwd(), "local_storage"))
        print("Local Storage Path -> ", self.profile.persistentStoragePath())

    def initUI(self):
        self.setLayout(self.layout)
        button = QPushButton("display cookie")
        button.clicked.connect(self.on_click)
        self.webView.

    def on_click(self):
        js = """
            cookieVal = document.cookie;
            document.getElementById("console").value = cookieVal;
            """
        result = js2py.eval_js(js)

    def saveMap(self):
        m = folium.Map(zoom_start=14, location=(37.377223, -121.990215))
        html = f"""
                <input type="text" value="" id="userInput" size="50">
                <button onclick="download()">Save</button><br>
                Console: <input type="text" value="" id="console" size="50">
                """
        folium.Marker(location=(37.391200, -122.001061), popup=html).add_to(m)

        java = folium.MacroElement().add_to(m)
        java._template = jinja2.Template("""    
            {% macro script(this, kwargs) %}

            function download() {
                var user_input = document.getElementById("userInput");
                document.cookie = user_input.value;
            }

            {% endmacro %}
        """)

        m.save('map.html')

    def placeMap(self):
        url = QUrl.fromLocalFile(os.path.join(os.getcwd(), 'map.html'))
        self.webView.load(url) 
        self.layout.addWidget(self.webView)

def main():
    app = QApplication([])

    window = DeGUI()
    window.initUI()
    window.saveMap()
    window.placeMap()

    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()