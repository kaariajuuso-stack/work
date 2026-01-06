import random
import time
from datetime import datetime
import haravasto

ruudukko = {
    "korkeus": None,
    "leveys": None,
    "koko": None,
    "x": None,
    "y": None,
    "miinat": None
}

tila = {
    "kentta": [],
    "kenttakopio": [],
    "jaljella": []
}

peli = {
    "voitto": False,
    "klikkaus": 0,
    "alku": None,
    "aika": None
}

def miinoita(miinoitettava_kentta, vapaiden_ruutujen_lista, asetettavien_miinojen_lukumaara):
    """
    Asettaa kentälle N kpl miinoja satunnaisiin paikkoihin.
    """ 
    for _ in range(asetettavien_miinojen_lukumaara):
        valittu_ruutu = random.choice(vapaiden_ruutujen_lista)
        x_koord, y_koord = valittu_ruutu
        miinoitettava_kentta[y_koord][x_koord] = "x"
        vapaiden_ruutujen_lista.remove(valittu_ruutu)

def kasittele_hiiri(x, y, nappi, muokkausnapit):
    """
    Tätä funktiota kutsutaan kun käyttäjä klikkaa sovellusikkunaa hiirellä.
    """
    ruudukko["x"], ruudukko["y"] = int(x/40), int(y/40)
    peli["klikkaus"] += 1
    tulvataytto(tila["kenttakopio"], ruudukko["x"], ruudukko["y"])
    havisit_pelin()
    voittoehto()
    
def peli_loppunut():
    """
    Näyttää pelaajalla kentän miinoineen pelin päätyttyä. 
    Sen jälkeen ikkuna sulkeutuu klikkaamalla.
    """
    tila["kenttakopio"].clear()
    def sulje(x, y, nappi, modit):
        haravasto.lopeta()
    haravasto.aseta_hiiri_kasittelija(sulje)
    
def havisit_pelin():
    """
    Funktio suoritetaan, jos pelaaja häviää. 
    """
    if tila["kentta"][ruudukko["y"]][ruudukko["x"]] == "x":
        peli_loppunut()

def voittoehto():
    """
    Funktio suoritetaan, jos pelaaja voittaa.
    Toteutuu, jos avaamattomia ruutuja on saman verran kuin miinoja.
    """
    avaamattomat_ruudut = 0
    for i in tila["kenttakopio"]:
        for j in i:
            avaamattomat_ruudut += j.count(" ")       
    if avaamattomat_ruudut == ruudukko["miinat"]:
        peli["voitto"] = True
        peli_loppunut() 
        print("Voitit pelin!")
        
def piirra_kentta():
    """
    Käsittelijäfunktio, joka piirtää kaksiulotteisena listana kuvatun miinakentän
    ruudut näkyviin peli-ikkunaan. Funktiota kutsutaan aina kun pelimoottori pyytää
    ruudun näkymän päivitystä.
    """
    haravasto.tyhjaa_ikkuna()
    haravasto.piirra_tausta()
    haravasto.aloita_ruutujen_piirto()
    for y, rivi_piir in enumerate(tila["kentta"]):
        for x, ruutu_piir in enumerate(rivi_piir):
                haravasto.lisaa_piirrettava_ruutu(ruutu_piir, x*40, y*40)
    for y, rivi_piir in enumerate(tila["kenttakopio"]):
        for x, ruutu_piir in enumerate(rivi_piir):
                haravasto.lisaa_piirrettava_ruutu(ruutu_piir, x*40, y*40)
    haravasto.piirra_ruudut()    
                   
def miinalaskuri(x_koordinaatti, y_koordinaatti, alue):
    """
    Laskee miinojen määrän tietyn ruudun kohdalta.
    """
    korkeus = len(alue)
    leveys = len(alue[0])
    miinat = 0

    if 0 <= x_koordinaatti < leveys and 0 <= y_koordinaatti < korkeus:
        if alue[y_koordinaatti][x_koordinaatti] == "x":
            miinat += 1
        for x_muutos in [-1, 0, 1]:
            for y_muutos in [-1, 0, 1]:
                if x_muutos == 0 and y_muutos == 0:
                    continue
                uusi_x = x_koordinaatti + x_muutos
                uusi_y = y_koordinaatti + y_muutos
                if 0 <= uusi_x < leveys and 0 <= uusi_y < korkeus and alue[uusi_y][uusi_x] == "x":
                    miinat += 1
    return miinat
    
def miinantunnistin():
    """
    Merkkaa kuinka monta miinaa kunkin ruudun ympäriltä löytyy.
    """
    for y, rivi in enumerate(tila["kentta"]):
        for x, ruutu in enumerate(rivi):
            if ruutu == " ":
                tila["kentta"][y][x] = str(miinalaskuri(x, y, tila["kentta"]))
        
def tulvataytto(alue, x, y):
    """
    Täyttää alueen rekursiivisesti nollilla alkavasta pisteestä.
    """
    korkeus = len(alue)
    leveys = len(alue[0])
    if 0 <= x < leveys and 0 <= y < korkeus and alue[y][x] == " ":
        lista = [(y, x)]
        y_lista, x_lista = lista.pop()
        alue[y_lista][x_lista] = str(miinalaskuri(x, y, tila["kentta"]))
        if alue[y][x] == "0":
            for x_muutos in [-1, 0, 1]:
                for y_muutos in [-1, 0, 1]:
                    uusi_x = x + x_muutos
                    uusi_y = y + y_muutos
                    tulvataytto(alue, uusi_x, uusi_y)      

def ajastin(alku):
    """
    Mittaa pelin keston minuuteissa.
    """
    loppu_aika = time.time()
    peli["aika"] = round((loppu_aika - alku)/60, 2)
    
def ajastin_alku():
    """
    Aloittaa pelin keston mittaamisen.
    """
    peli["alku"] = time.time()
    
def tiedosto_paivitys():
    """
    Päivittää tiedostoon tilastodataa.
    """
    tiedosto = open("tulokset.txt", "a")
    aika = datetime.now()
    tiedosto.write(f"{aika.strftime('%d/%m/%Y %H:%M:%S')},{(peli['voitto'])},{(peli['aika'])},\
{peli['klikkaus']},{ruudukko['koko']},{ruudukko['miinat']}\n")

def luo_kentta():
    """
    Kysyy pelaajalta tarvittavat arvot. Kun pelaaja on antanut tarvittavat arvot, funktio luo tarvittavat listat
    kentän toteuttamiseen.
    """
    while True:
        try:
            korkeus = int(input("Anna kentän korkeus: "))
            if korkeus > 0:
                break
            else:
                print("Yritä uudelleen")
        except ValueError:
            print("Yritä uudelleen")
    while True:
        try:
            leveys = int(input("Anna kentän leveys: "))
            if leveys > 0:
                break
            else:
                print("Yritä uudelleen")   
        except ValueError:
            print("Yritä uudelleen")
    while True:
        try:
            miinat = int(input("Anna miinojen määrä: "))
            if miinat >= leveys * korkeus:
                print("Miinoja pitää olla vähemmän!")
            elif miinat > 0:
                break
            else:
                print("Yritä uudelleen")   
        except ValueError:
            print("Yritä uudelleen")
    ruudukko["korkeus"] = korkeus
    ruudukko["leveys"] = leveys
    ruudukko["koko"] = ruudukko["leveys"] * ruudukko["korkeus"]
    ruudukko["miinat"] = miinat
    kentta = []
    for rivi in range(ruudukko["korkeus"]):
        kentta.append([])
        for sarake in range(ruudukko["leveys"]):
            kentta[-1].append(" ")
    tila["kentta"] = kentta
    
    jaljella = []
    for x in range(ruudukko["leveys"]):
        for y in range(ruudukko["korkeus"]):
            jaljella.append((x, y))
            
    tila["jaljella"] = jaljella    
    
    kenttakopio = kentta.copy()
    kenttakopio.clear()
    for rivi in range(ruudukko["korkeus"]):
        kenttakopio.append([])
        for sarake in range(ruudukko["leveys"]):
            kenttakopio[-1].append(" ")

    tila["kenttakopio"] = kenttakopio
    