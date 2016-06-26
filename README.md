# server_lader

## Umbau eines ESP120 zu einem Ladegerät

### Trivia:
Das ESP120 ist ein 3kW starkes Servernetzteil mit einer Ausgangsspannung von standardmäßig 52V und bis zu >50A. Das Netzteil lässt sich allerdings leicht auf andere Spannungen umrüsten, um z.B. als Akkulader oder sehr potentes Labornetzteil eingesetzt zu werden.


### Was tut dieses Projekt?
dieses Projekt soll die zur Spannungsverstellung nötige Hilfsspannung aus den vom Netzteil zusätzlich bereitgestellten 5V erzeugen, die Ausgangsspannung regeln und für einen Ladestrom nach einem Ladeprogramm, das extern eingespeist wird, einstellen. Das Netzteil muss nur minimal gemoddet werden.


### Der Mod:
siehe elweb, eigene Bilder liegen im Unterordner "Bilder".
Einfach gesagt, werden an den beiden Pads des Potis "RV..." Kabel angelötet, die dann mit der Steuerplatine verbunden werden. Hier wird die Sollwertspannung eingespeist.

Es existieren Messwerte für 3 Sollwertspannungen:

* 61V 35,4 kOhm (-1,5V am Poti)
* 60V 42,5 kOhm (-1,3V am Poti)
* 59V 51,6 kOhm (-1,1V am Poti)


## Umbau auf eine andere Spannung für die Überspannungsabschaltung
Das Netzteil besitzt zusätzlich zur Regelspannung, die am Poti zu verändern ist, eine feste Spannung, bei der eine Überspannungsabschaltung stattfindet. Die soll das Netzteil und die angeschlossenen Verbraucher schützen. Teilweise ist es jedoch nötig, nahe an 60V zu kommen, beispielsweise, wenn man einen 48V-Bleiakkupack laden will. Daher ist in diesen Fällen die Überspannungsabschaltung auf einen leicht höheren Spannungswert zu verschieben. 

Dazu ist es notwendig einen zusätzlichen Widerstand parallel zu den bereits vorhandenen auf einer der Platinen einzulöten. Es handelt sich um einen SMD-Widerstand in Bauform 0805 mit dem Wert "473" also 47 kOhm, der direkt neben einem SMD-Bauteil im Gehäuse SOT-223-3 liegt. Im Ordner "Bilder" ist die Stelle zu sehen, wo der Widerstand eingelötet werden muss.

Wenn man über die bereits verbauten Widerstände im eingebauten Zustand misst, kommt man auf 4,096 Ohm. Mit einem parallel geschaltenen 51,6 kOhm Widerstand kommt man so auf einen Widerstand von 3,79kOhm

Es wird hier allerdings ein Widerstand mit 47 kOhm empfohlen, da mit diesem 60V gerade so erreicht werden, aber die Kondensatoren des Netzteils noch nicht beschädigt werden. Diese halten 63V aus. Für noch höhere Ausgangsspannungen müssten die Kondensatoren durch andere Typen mit 80V Festigkeit getauscht werden. Zusätzlich wird das Netzteil ab ca 61-62V instabil.


## Änderungen für Hardwareversion 2:
 
* Footprint für Stecker anpassen
* zusätzlicher Kondensator für AREF
* Micromatch-Stecker auf Display Platine auf Unterseite verlegen
* zweiter Stecker für I2C, um das OLED anzuschließen (JST XH, stehend)


## TODOs Software:

* I2C-Auswertung von Lüfter und Temperaturen
* Umbau auf so ein kleines OLED-Display z.b. mit der U8g-Lib

