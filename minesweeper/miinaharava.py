import haravasto
import miinamoottori as m

if __name__ == "__main__":
    silmukka = True
    
    def valikko():
        """
        Pelin alkuvalikko.
        """
        valinta = input("Haluatko aloittaa pelin (p), lopettaa (q) vai katsoa tilastot (s): ")
        if valinta.lower() == "q":
            return False
        if valinta.lower() == "p":
            m.luo_kentta()
        
            haravasto.lataa_kuvat("spritet")
            haravasto.luo_ikkuna(m.ruudukko["leveys"]*40, m.ruudukko["korkeus"]*40)
            m.miinoita(m.tila["kentta"], m.tila["jaljella"], m.ruudukko["miinat"])
            m.miinantunnistin()
            haravasto.aseta_piirto_kasittelija(m.piirra_kentta)
            haravasto.aseta_hiiri_kasittelija(m.kasittele_hiiri)
            m.ajastin_alku()
            haravasto.aloita()
        
            m.ajastin(m.peli["alku"])
            m.tiedosto_paivitys()
            return True
        if valinta.lower() == "s":
            with open("tulokset.txt", "r", encoding="utf-8") as tiedosto:
                for teksti in tiedosto:
                    teksti_uusi = teksti.strip("\n") 
                    tulos = teksti_uusi.split(",")
                    if tulos[1] == "True":
                        print(f"{tulos[0]} Voitit pelin! Aikasi: {tulos[2]} minuuttia, Siirtojesi määrä: {tulos[3]}, Kentän koko: {tulos[4]}, Miinat: {tulos[5]}")
                    else:
                        print(f"{tulos[0]} Hävisit pelin! Aikasi: {tulos[2]} minuuttia, Siirtojesi määrä: {tulos[3]}, Kentän koko: {tulos[4]}, Miinat: {tulos[5]}")  
                return True
        else:
            print("Yritä uudelleen")
            return True
    while silmukka:
        silmukka = valikko()
