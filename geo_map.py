# https://levelup.gitconnected.com/creating-interactive-maps-with-python-folium-and-some-html-f8ac716966f
import folium
import base64 
import jinja2

center = [37.377223, -121.990215]
map_AILSV = folium.Map(location=center, zoom_start=15, control_scale=True)

img = '20230113_154702.gif'
encoded = base64.b64encode(open(img, 'rb').read()).decode()
gif_html = f'<img src="data:image/gif;base64,{encoded}">'

html = f"""{gif_html}
        <br>Description: <input type="text" id="myInput"><br>
        <button onclick="myFunction()">Save</button>
        """

folium.Marker(location=[37.391200, -122.001061], popup=html).add_to(map_AILSV)

java = folium.MacroElement().add_to(map_AILSV)
java._template = jinja2.Template("""
    {% macro script(this, kwargs) %}
    function myFunction() {
        var copyText = document.getElementById("myInput");

        copyText.select();
        copyText.setSelectionRange(0, 99999); // For mobile devices

        navigator.clipboard.writeText(copyText.value);

        alert("Copied the text: " + copyText.value);
    }
    {% endmacro %}
""")

map_AILSV.save('map.html')