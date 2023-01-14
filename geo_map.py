# https://levelup.gitconnected.com/creating-interactive-maps-with-python-folium-and-some-html-f8ac716966f
import folium, io, sys
import base64
from folium import IFrame 

import sys
import io
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView # pip install PyQtWebEngine









center = [37.377223, -121.990215]
map_AILSV = folium.Map(location=center, zoom_start=15, control_scale=True)

test_image = '20230113_154702.gif'
encoded = base64.b64encode(open(test_image, 'rb').read()).decode()

html = '<img src="data:image/gif;base64,{}">'.format
iframe = IFrame(html(encoded), width=600, height=600)
popup = folium.Popup(iframe, max_width=600)

folium.vector_layers.Marker(location=[37.391200, -122.001061], tooltip='DE #5', popup = popup).add_to(map_AILSV)

map_AILSV.save('index.html')