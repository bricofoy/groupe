/****************************************************************************************
 * 		Gestion automatique de groupe électrogène.				*
 ****************************************************************************************
  (c) bricofoy (bricofoy@free.fr) 2012
  Programme sous licence GNU GPL
  
  
  Démarrage sur commande à distance ou locale.
  Arrêt sur fin de commande à distance, coupure contact ou nouvelle commande locale.
  Arrêt avec alarme sur défaut pression huile. Redémarrage seulement si reset manuel.
  #Alarme si temps maintenance dépassé.
  Alarme si défaut de charge batterie de démarrage.
  
  
  Historique des modifications :
    DATE	| 	AUTEUR		| Ver.	| 	MOTIF
  ------------------------------------------------------------------------------------
  01-10-2012	| bricofoy@free.fr	| 0.1	| création
  09-10-2012	| bricofoy@free.fr	| 0.5	| change le comportement de ve_loc, correction bug tempoMS()
		|			|	| ajout second timer tempoMS2()
  15-10-2012	| bricofoy@free.fr	| 0.8	| modifs pins pour correspondre au schéma
		|			|	| suppression pin mesure ubat (même pin que alim, plus simple)
  25-10-2012	| bricofoy@free.fr	| 0.9	| ajustements pour correspondre au PCB 1.1
  26-10-2012	| bricofoy@free.fr	| 0.9.1 | ajout visu et reset défauts
  08-02-2013    | bricofoy@free.fr      | 0.9.9 | suppression erreur temps maintenance + plein d'autres trucs
                |                       |       | ajout test si ve_alim avant vérifier ubat pour éviter fausse erreur
                |			|	| VERSION QUI FONCTIONNE !!
  22-03-2013	| bricofoy@free.fr	|	| Bug au démarrage ! La lecture entrée alim tombe et ça coupe --> en fait
		|			|	| c'est la batterie qui baisse trop. Seuil de détection alim baissé à 3V.
  30-04-2013	| bricofoy@free.fr	| 1.0.0 | Ajout tempo de chauffage et refroidissement moteur + commande relais 
		|			|	| coupure sortie commandé par s_out 
  10-05-2013	|			|	| renommage s_dec --> s_out   vs_decmp --> vs_out
		|			|	| TODO:supprimer l'état et_decomp
		|			|	| ajout tempo sur détection défaut PrH pour éviter détection PrH au lieu de calage
  
  
  
 */


#include <EEPROM.h>

//broches de sortie
#define s_alim	13	//Maintien alim
#define s_ev	12	//EV contact
#define s_dem	10	//Demarreur
#define s_pre	 9	//Préchauffage
#define s_out	11	//Relais de coupure sortie puissance
#define s_alarme 7	//Témoin d'alerte
#define s_ok	 8	//Témoin OK

//valeurs des sorties
#define vs_alim	   1	//format : alim|EV|demarreur|prechaufage|sortie|0|alerte|ok
#define vs_ev	   2
#define vs_dem	   4
#define vs_prech   8
#define vs_out    16
#define vs_alarme 64
#define vs_ok	 128

//broches d'entrées
#define e_alim_ubat	A0	//Alim et tension batterie
#define e_ext		A1	//Commande externe
#define e_prh		2	//Pression huile
#define e_run		A3	//1 si moteur en route
#define e_rst		3	//entrée reset alarme
#define e_loc		A2	//entrée commande locale

//valeurs des entrées
#define ve_alim	1	//format : alim|commande_ext|pression_huile|run|reset|commande_locale|0|0
#define ve_ext	2
#define ve_prh	4
#define ve_run	8
#define ve_rst	16
#define ve_loc	32

//valeurs de l'état sauvegardé en eeprom
#define vet_ok	    0	//fonctionnement ok
#define vet_defchg  1	//défaut charge batterie
#define vet_tmax    2	//temps max dépassé
#define vet_defph   4	//défaut pression huile
#define vet_defdem  8	//défaut démarrage
#define vet_defcal  16	//défaut calage
#define vet_defubat 32	//défaut batterie faible



//adresses de stockage en eeprom
#define adr_etat  	0	//registre etat precedent
#define adr_tpsLSB  	1	//temps de fonctionnement depuis raz temps. ATTENTION stockage sur 2 bytes
#define adr_tpsMSB  	2
#define adr_tpsm 	3	//minutes du temps de fonctionnement.
#define adr_tpstotLSB 	4	//temps de fonctionnement total. ATTENTION stockage sur 2 bytes
#define adr_tpstotMSB 	5
#define adr_tpstotm	6	//minutes du temps de fonctionnement total.

//états de la machine à états
#define et_attente	1
#define et_prechauffe	2
#define et_demarreur	3
#define et_run		4
#define et_defaut	5
#define et_pause_dem	6
#define et_off		7
#define et_calage	8
#define et_decomp	9
#define et_manuel	10
#define et_force	11
#define et_pre_run	12
#define et_post_run	13

//valeur du coef pont diviseur mesure ubat
#define coef_ubat 	0.0205394

#define min_ubat 	10.8		//Tension minimale en dessous de laquelle on passe en défaut pour protéger le démarreur
#define min_chg		13		//Tension en dessous de laquelle on détecte un défaut de charge batterie
#define min_ubat_reset  12		//Tension au dessus de laquelle le défaut ubat disparait tout seul

//temporisations, valeurs en ms
#define tempo_attente		180E3	//3 min
#define tempo_attente_defaut	7.2E6	//7,2*10^6ms = 2heures
#define tempo_prechauffage	0.5E3	//1 secondes
#define tempo_decomp		0.5E3	//0.5 s
#define tempo_demarreur		6E3	//6 secondes
#define tempo_attente_calage	5E3	//5 secondes
#define tempo_fin_calage	10E3	
#define tempo_compteur		2E3
#define tempo_manuel		18E5	//30 minutes
#define tempo_valide_local	3E3
#define tempo_valide_externe	1E3
#define tempo_valide_reset	5E3
#define tempo_montre_reset	2E3
#define tempo_montre_defaut	2E3
#define tempo_reset_montre_def	2E3
#define tempo_pause_dem		15E3
#define tempo_detection_calage  3E3
#define tempo_detection_defph	0.5E3
#define tempo_coupe_alim	0.5E3
#define tempo_pre_run		60E3	//tempo chauffage moteur avant activation sortie
#define tempo_post_run		180E3	//tempo refroidissement après coupure sortie avant coupure moteur
#define tempo_cligno		0.6E3

#define max_cpt_calage		1	//nombre d'essais de redémarrage après calage
#define max_cpt_dem		2	//nombre d'essais de démarrage

#define tps_maintenance		30	//heures


//variables globales
byte entrees=0;			//registre avec les valeurs des entrées
byte sorties=0;			//registre avec valeurs des sorties
byte etat = 0;			//registre d'état du système

float ubat=0;			//tension batterie

unsigned long temps_courant=0;	//temps depuis mise sous tension, en millisecondes
unsigned int tps=0;			//temps depuis dernière RAZ compteur, en heures
byte tpsm=0;			//minutes du temps ci dessus
unsigned int tpstot=0;		//temps depuis installation, en heures
byte tpstotm=0;			//minutes de tpstot

byte etat_machine=et_attente;	//faut-il préciser ce que c'est ?

boolean verbose = false;	//définit si on raconte des trucs sur le port série ou non
unsigned long tempo_force;
boolean force_eeprom = false;
boolean force_entrees = false;


void setup() {                
  // initialisations

  Serial.begin(9600);
  
  pinMode(s_alim,  OUTPUT);
  pinMode(s_alarme,OUTPUT);
  pinMode(s_pre,   OUTPUT);
  pinMode(s_dem,   OUTPUT); 
  pinMode(s_out,   OUTPUT);
  pinMode(s_ok,    OUTPUT);
  pinMode(s_ev,    OUTPUT);
  
  pinMode(e_alim_ubat, INPUT);
  pinMode(e_ext,  INPUT); 
  pinMode(e_prh,  INPUT);
  //digitalWrite(e_prh, HIGH); 			//Active la resistance de pull-up interne
  //pinMode(e_ubat, INPUT);
  pinMode(e_run,  INPUT);
  pinMode(e_rst,  INPUT);
  pinMode(e_loc,  INPUT);
  
  digitalWrite(s_alim, HIGH); 			//Auto-maintien du relais d'alimentation
  
  etat = EEPROM.read(adr_etat);			//Lecture de l'état précédent
  tps = EEPROM.read(adr_tpsMSB)<<8;		//lecture temps depuis raz, lecture MSB
  tps += EEPROM.read(adr_tpsMSB);		//tps, lecture du LSB
  tpsm = EEPROM.read(adr_tpsm);			//minutes de tps
  tpstot = EEPROM.read(adr_tpstotMSB)<<8; 	//temps fonctionnement total depuis installation, lecture du MSB
  tpstot += EEPROM.read(adr_tpstotLSB);		//tpstot, lecture du LSB
  tpstotm = EEPROM.read(adr_tpstotm);		//minutes de tpstot


  Serial.println("reset  ");
  Serial.println(" ");

  sorties = vs_alim+vs_alarme+vs_ok;
  ecriture_sorties();
  delay(500);
}


void lecture_entrees() { 
  // lecture des entrées, stockage dans une seule variable byte pour economiser de la ram et faciliter les manipulations
float externe = 0;
  
  ubat = coef_ubat * analogRead(e_alim_ubat);
  externe = analogRead(e_ext);
  
  if (!force_entrees)
  {    
    entrees = ve_alim * (ubat > 2);		//si ubat>5 alors l'expression vaut TRUE, ou  1
    if (externe > 10)
      entrees += ve_ext ;
    entrees += ve_prh * !digitalRead(e_prh);	// prh et run sont en logique inversée sur la carte
    entrees += ve_run * !digitalRead(e_run);
    entrees += ve_rst * digitalRead(e_rst);
    entrees += ve_loc * digitalRead(e_loc);
  }

  temps_courant = millis();

  
}

void ecriture_sorties() {
  static boolean flagep=false;


  //activation des sorties selon les valeurs des bits du registre sorties
  if ((!(sorties & vs_alim))||force_eeprom)
  {					//Ecrit les variables d'état en eeprom seulement si coupure alim ou forçage demandé
    if (!flagep) {			//pour limiter les écritures en eeprom
      EEPROM.write(adr_etat, etat);	//"flagep" est là pour écrire une seule fois en eeprom si on passe plusieurs fois dans la fonction     
      EEPROM.write(adr_tpsLSB, (byte)tps); 	//avant que l'alim tombe réellement.
      EEPROM.write(adr_tpsMSB,(byte)(tps>>8));
      EEPROM.write(adr_tpsm, tpsm);		
      EEPROM.write(adr_tpstotLSB,(byte)tpstot);
      EEPROM.write(adr_tpstotMSB,(byte)(tpstot>>8));
      EEPROM.write(adr_tpstotm, tpstotm);
      delay(50);
      flagep = true;
      force_eeprom = false;
      Serial.print("\nEcriture EEPROM faite\n");
    }
  }
  if ((sorties & vs_alim) && flagep)    //au cas où, mais normalement on ne rentrera jamais ici
    flagep = false;
  
  digitalWrite(s_alim,	( sorties & vs_alim	));
  digitalWrite(s_ev,  	( sorties & vs_ev	));
  digitalWrite(s_dem, 	( sorties & vs_dem	));
  digitalWrite(s_pre, 	( sorties & vs_prech	));
  digitalWrite(s_out,  	( sorties & vs_out	));
  digitalWrite(s_alarme,( sorties & vs_alarme	));
  digitalWrite(s_ok, 	( sorties & vs_ok	));
}

void liaison_serie() {
  //écriture sur le port série
  unsigned int valeur=0;
  char commande=0;
  static byte etat_machine_precedent=0;
  
  if (etat_machine != etat_machine_precedent)
  {
    Serial.print("etat : ");
      switch (etat_machine) {
	case et_attente : Serial.println("et-attente"); break;
	case et_calage  : Serial.println("et_calage"); break;
	case et_decomp  : Serial.println("et_decomp"); break;
	case et_defaut  : Serial.println("et_defaut"); break;
	case et_demarreur : Serial.println("et_demarreur"); break;
	case et_force   : Serial.println("et_force"); break;
	case et_manuel  : Serial.println("et_manuel"); break;
	case et_off     : Serial.println("et_off"); break;
	case et_pause_dem : Serial.println("et_pause_dem"); break;
	case et_post_run : Serial.println("et_post_run"); break;
	case et_pre_run : Serial.println("et_pre_run"); break;
	case et_prechauffe : Serial.println("et_prechauffe"); break;
	case et_run     : Serial.println("et_run"); break;
	default : Serial.println("inconnu"); break;
	}
  }
  
  etat_machine_precedent = etat_machine;
  
  if(verbose)
  {
    Serial.print("entrees = ");
    Serial.print(entrees,BIN);
    Serial.print(" ");
    Serial.println(entrees);

    Serial.print("etat machine = ");
    Serial.print(etat_machine);
    Serial.println("  ");
    //if (etat_machine == et

    Serial.print("sorties = ");
    Serial.print(sorties,BIN);
    Serial.print(" ");
    Serial.println(sorties);
    Serial.print("tps : ");
    Serial.print(tps);
    Serial.print(":");
    Serial.print(tpsm);
    Serial.print(" ");
    Serial.print(tpstot);
    Serial.print(":");
    Serial.print(tpstotm);
    Serial.print(" ubat=");
    Serial.println(ubat);
    Serial.print("etat : ");
    Serial.print(etat,BIN);
    Serial.print(" ");
    Serial.println(etat);
    Serial.println(" ");
  }




  while (Serial.available())
  {
    commande = Serial.read();
    delay(50);					//delai pour permettre la bonne lecture du port série.
    valeur=((word((Serial.read()-48)))*100); 	//-48 car la valeur ascii du 0 est 48, celle du 1 est 49, etc.
    delay(50);					//et*100 car on lit d'abord les centaines
    valeur+=(byte)((Serial.read()-48)*10);	//puis les dizaines
    delay(50);
    valeur+=(byte)((Serial.read()-48));		//puis les unités

    Serial.print("\ncommande :");
    Serial.print(commande);
    Serial.print("\nvaleur :");
    Serial.println(valeur, DEC);
    
    switch ( commande ) {
      case 'R' : 				//"R" reset tout
	etat = vet_ok;
	etat_machine = et_attente;
	tempoMS(0);
	tempoMS2(0);
	tps = 0;
	tpsm = 0;
	tpstot = 0;
	tpstotm = 0;
	force_entrees = false;
	sorties = vs_alim;
	ecriture_sorties();
	delay(100);
	break;
      case 'r' :				//reset tout sauf tps total
	etat = vet_ok;
	etat_machine = et_attente;
	tempoMS(0);
	tempoMS2(0);
	tps = 0;
	tpsm = 0;
	force_entrees = false;
	sorties = vs_alim;
	ecriture_sorties();
	delay(100);
	break;
      case 'e' : 				//force valeur etat
	etat = valeur;
	break;
      case 'h' :				//"hxxx"force la valeur de tps à xxx
	tps = (unsigned int)valeur;
	break;
      case 'H' :				//"Hxxx" force la valeur de tpstot à xxx
	tpstot = (unsigned int)valeur;
	break;
      case 'm' :
	tpsm = valeur;
	break;
      case 'M' :
	tpstotm = valeur;
	break; 
      case 'W' :				//force une écriture en eeprom
	force_eeprom = true;
	break;
      case 't' : 				//"t0xx" force état machine à xx
	etat_machine = valeur;
	tempoMS(0);
	tempoMS2(0);
	break;
      case 'E' :				//"Exxx" force les entrées à xxx
	if (force_entrees)
	  force_entrees = false;
	else
	{
	  force_entrees = true;
	  entrees = valeur;
	}
	
	break;
      case 'F' : 				//"Fxxx" Mode de forçage des sorties pour xxx secondes
	etat_machine = et_force;
	tempo_force = (unsigned long)valeur * 1E3;
	tempoMS(0);
	tempoMS2(0);
	break;
      case 'S' : 				//"Syyy" force les sorties à yyy, à utiliser avec Fxxx
	sorties = valeur;
	break;
      case 'V' : 				//"V" verbose
      case 'v' :
	if(verbose) verbose = false;
	else verbose = true;
	break;
      case 'p' :
	Serial.print("Sorties = ");
	Serial.println(sorties);
	Serial.print("Entrees = ");
	Serial.println(entrees);
	
	break;
      case '?' :
	Serial.print("\nListe des commandes :\nR    reset tout\nr    reset tout sauf tps total\nexxx force valeur etat a xxx\nExxx force les entrees a xxx\nFxxxSyyy force les sorties a yyy pour xxx secondes\nW    force une ecriture en eeprom\nt0xx force etat machine a xx\nhxxx force la valeur de tps a xxx\nHxxx force la valeur de tpstot a xxx\nV ou v mode verbeux\n?    affiche ce message");
      default :
	break;
    }
    
  }
}



byte tempoMS(unsigned long duree) {
  //comme son nom l'indique... fournit une temporisation en ms
  
  //le premier appel avec "duree" lance la tempo, retourne false
  //ensuite retourne false si "duree" (en ms) n'est pas écoulée, puis true lors de son 
  //premier appel après écoulement de la durée "duree"
  //l'appel suivant relance la tempo.
  
  //un appel avec "duree" à 0 termine la tempo et retourne true.


  //TODO idée pour simplifier : virer le reset auto quand le temps est écoulé,
  //uniquement reset manuel -> permet des appels avec temps différents depuis le même état de la machine
  
  static unsigned long temps_debut = 0;
  //static unsigned long duree_debut = 0;
  static boolean tempo = false;
    
  if (!duree) {	tempo = false;
		return true; }
     
  if (!tempo) {	tempo = true;
		temps_debut = temps_courant;
		//duree_debut = duree;
		return false; }
  else 
    if (temps_courant > (temps_debut + duree)) {
		tempo = false;				//remet à 0 le flag tempo si durée écoulée pour permettre le lancement suivant
		return true;
    }
    else return false;
}

byte tempoMS2(unsigned long duree) {
  static unsigned long temps_debut = 0;
  static boolean tempo = false;
  if (!duree) {	tempo = false;
		return true; }
  if (!tempo) {	tempo = true;
		temps_debut = temps_courant;
		return false; }
  else 	if (temps_courant > (temps_debut + duree)) {
		tempo = false;
		return true; }
	else return false;
}

byte tempoMS3(unsigned long duree) {
  static unsigned long temps_debut = 0;
  static boolean tempo = false;
  if (!duree) {	tempo = false;
  return true; }
  if (!tempo) {	tempo = true;
  temps_debut = temps_courant;
  return false; }
  else 	if (temps_courant > (temps_debut + duree)) {
    tempo = false;
    return true; }
    else return false;
}

byte tempoMS4(unsigned long duree) {
  static unsigned long temps_debut = 0;
  static boolean tempo = false;
  if (!duree) {	tempo = false;
  return true; }
  if (!tempo) {	tempo = true;
  temps_debut = temps_courant;
  return false; }
  else 	if (temps_courant > (temps_debut + duree)) {
    tempo = false;
    return true; }
    else return false;
}



void incremente_tps() {
  //compte les minutes et heures de fonctionnement tant que la fonction est appelée toutes les tempo_compteur millisecondes ou moins
  static unsigned long temps_debut=0;
  static unsigned long temps_precedent=0;

  if (!(temps_debut)) {						//lance le compteur lors du premier appel depuis un arret
    temps_debut = temps_courant;
    temps_precedent = temps_courant;
  }

  
  if (temps_courant > (temps_precedent + tempo_compteur)) {	//stoppe le compteur si plus de tempo_compteur entre deux appels successifs
    temps_precedent = 0;
    temps_debut = 0;
    return;
  }
  else temps_precedent = temps_courant;

  if (temps_courant > (temps_debut + 60E3)) {			//incrémente le compteur toutes les minutes
    temps_debut += 60E3;
    tpsm ++;
    tpstotm ++;
  }

  if (tpsm > 59) { tps++;					//incrémente tps et tpstot toutes les 60 minutes comptées
		   tpsm = 0; }
  if (tpstotm > 59) { tpstot++;
		      tpstotm=0; }
}
  

    
void machine_etat() {
  
  //gestion du moteur
 
  static byte cpt_calage=0;
  static byte cpt_dem=0;
  static boolean flag=0,flag2=0,flagrun=0,flagext=0,flagalim=0;
  static boolean local=0,manuel=0;


  
  switch (etat_machine) 
  {    
    case et_off:  //*************************************************************************************************************
	    sorties = 0;
            //if(entrees & ve_alim)
            //  etat_machine = et_attente;
	    break;
	    
    case et_force:  //*************************************************************************************************************
	    if (tempoMS(tempo_force))
	      etat_machine = et_attente;
	    break;  //TODO : sauver état précédent et y revenir au lieu de et_attente
	    
	   
    case et_attente: //*************************************************************************************************************
	    sorties = vs_alim;

	    if (!(flag||flag2))
	    {		//pour pas que ça soit le bordel quand on est mode lecture défaut, on ne passe pas là dedans

	      if ((entrees & ve_rst)&&(entrees & ve_loc))	//passage mode manuel
	      {
		etat_machine = et_manuel;
		tempoMS(0);
		tempoMS2(0);
		break;
	      }

	      
	      if (etat & (vet_defdem|vet_defcal|vet_defph|vet_defubat))
	      {
		tempoMS(0);
		tempoMS2(0);
		etat_machine = et_defaut;			//passage directement en défaut si il y a un défaut grave mémorisé
		break;
	      }

	      if ((etat & vet_defchg)||(etat & vet_tmax))
		sorties = vs_alim+vs_alarme;			//si défaut de charge mémorisé, alarme
 
	      if ((entrees & ve_alim)&&(ubat < min_ubat)) 	//on passe en défaut si ubat trop faible
	      {
		etat_machine = et_defaut;
		tempoMS(0);
		tempoMS2(0);
		if (!(etat & vet_defubat))
		  etat +=  vet_defubat;
		break;
	      }
	      else if (etat & vet_defubat)
		      etat -= vet_defubat;


	      if (tempoMS(tempo_attente))
	      {							//coupe l'auto-maintien de l'alim si on dépasse la tempo de veille
		etat_machine = et_off;
		sorties = 0;
		break;
	      }

	    }

	    

// partie lecture défaut
		  


	    if (etat&&(entrees & ve_rst)&&(!flag)) 
            {
	      //if (!flag)
	//	tempoMS(0);
	       		//si reset appuyé pendant plus de tempo_montre_defaut, alors on affiche le défaut
		flag = 1;
	    }
	    
	    if (flag)						//on maintient allumée la led ok pendant tempo_montre_defaut
	      if (!(tempoMS2(tempo_montre_defaut))) {
		if (etat & vet_tmax)
		  sorties = vs_alim+vs_ok;			//défaut temps : juste ok allumée
		if (etat & vet_defchg)				//défaut charge : ok et al allumées
		  sorties = vs_alim+vs_ok+vs_alarme;
		break;		
	      }
	      else
		flag = 0;


	    

	    if (entrees & ve_ext )
	    {
	      flagext = true;
	      tempoMS(0);
	      if (tempoMS2(tempo_valide_externe))
	      {							//passe au préchauffage si demande départ externe
		etat_machine = et_prechauffe;
		flagext=0;
		tempoMS(0);
		tempoMS2(0);
		break;
	      }
	    }
	    else
	      if (flagext)
	      {
		flagext = 0;
		tempoMS2(0);
	      }


	    if (entrees & ve_loc )
	    {
	      flag2 = true;
	      tempoMS(0);
	      if (tempoMS2(tempo_valide_local))
	      {							//passe au préchauffage si demande départ interne
		etat_machine = et_prechauffe;			
		local = true;
		tempoMS(0);
		tempoMS2(0);
		flag2=0;
                break;
	      }
	    }
	    else
	      if (flag2)
		{
		  flag2 = 0;
		  tempoMS2(0);
		}


		      
	    if (!(entrees & ve_alim ))
	    {							//coupe l'auto-maintien de l'alim si l'alim locale est coupée
	      etat_machine = et_off;
	      sorties=0;
	    }
	    break;
      
    case et_prechauffe: //************************************************************************************************************************************
	    sorties = vs_alim+vs_ok+vs_prech;
	    
	    if (tempoMS(tempo_prechauffage))
	      etat_machine = et_decomp;

	    if ((entrees&ve_alim)&&(ubat < min_ubat))
	    {
	      etat_machine = et_defaut;
	      if (!(etat & vet_defubat))
		etat +=  vet_defubat;
	    }
	    
 	    if (!((entrees & ve_ext )||local)) {		//repasse en attente si fin demande départ externe
 	      etat_machine = et_off;
 	      tempoMS(0);					//remise à zéro du timer pour utilisation ultérieure
 	    }

	    if (!(entrees & ve_alim))
	    {
		etat_machine = et_off;				//coupure alim -> off
		tempoMS(0);
                tempoMS2(0);
	    }
	    
	    break;

    case et_decomp:  //***************************************************************************************************************
	    sorties = vs_alim+vs_ok+vs_prech+vs_ev+vs_dem;

	    if (!flag)
	    { //flag sert à vérifier si on passe pour la première fois ou non dans la fonction
	      flag=true;
	      //s'execute au premier passage dans cet état uniquement
	      if (etat & vet_defcal)		//si on essaye de redémarrer après un calage, on ne fait qu'un seul essai
		cpt_dem += max_cpt_dem;		//donc on incrémente d'au moins max_cpt_dem ainsi ça ne passe la condition suivante que
	      else cpt_dem++;			//si cpt_dem = 0 avant cette incrémentation.
	    }

	    //s'exécute dans les passages suivants dans cet état
	    if (cpt_dem > max_cpt_dem)
	    {
	      flag = false;
	      etat_machine = et_defaut;				//nombre max d'essais de démarrage dépassé -> défaut
	      cpt_dem = 0;					//remise à 0 du compteur pour la prochaine fois
	      tempoMS(0);
              break;
	    }
	    
	    if (tempoMS(tempo_decomp))
	    {
	      flag = false;
	      etat_machine = et_demarreur;
              break;
	    }

 	    if (!((entrees & ve_ext )||local))
 	    {							//repasse en attente si fin demande départ externe ou coupure alim
 	      etat_machine = et_attente;
 	      tempoMS(0);					//remise à zéro du timer pour utilisation ultérieure
	      flag = false;
	    }

	    if (!(entrees & ve_alim))
	    {
		etat_machine = et_off;				//coupure alim --> off
		tempoMS(0);
                tempoMS2(0);
		flag = false;
	    }
	    

	    break;
	
    case et_demarreur: //*************************************************************************************************************
  	    sorties = vs_alim+vs_ok+vs_prech+vs_ev+vs_dem;

	    if (tempoMS(tempo_demarreur))
	    {							//si temps max de fcnmt démarreur dépassé,
	      if (!(etat & vet_defdem))			
		etat += vet_defdem;				//on signale un défaut démarrage 
	      etat_machine = et_pause_dem;			//et on fait une pause pour refroidir le démarreur
	    }
		 
	    if (entrees & ve_run)
	    {							//le moteur a démarré, passage à l'état suivant
	      etat_machine = et_pre_run;
	      tempoMS(0);
              tempoMS2(0);
              break;
	    }


// 	    if (!((entrees & ve_ext )||local))
// 	    {							//repasse en attente si fin demande départ externe
// 	      if((etat & vet_defcal)||(etat & vet_defdem))
// 		etat_machine = et_defaut;			//repasse en défaut si on était en cours de démarrage après defaut calage ou démarrage
// 	      else etat_machine = et_attente;
// 	      tempoMS(0);					//remise à zéro du timer pour utilisation ultérieure
// 	    }
	    
	    if (!(entrees & ve_alim))
	    {
		etat_machine = et_off;			//repasse en off si coupure alim
		tempoMS(0);
	    }


	    break;

    case et_pause_dem:   //***********************************************************************************************************
	    sorties = vs_alim+vs_alarme+vs_ok+vs_prech+vs_ev;
	    
	    if (tempoMS(tempo_pause_dem))
	      etat_machine = et_decomp;

	    if ((entrees & ve_alim)&&(ubat < min_ubat)) 
	    {
	      tempoMS(0);
	      etat_machine = et_defaut;
	      if (!(etat & vet_defubat))
		etat +=  vet_defubat;
	    }

	    if (!((entrees & ve_ext )||local)) 		//repasse en défaut si fin demande départ externe avant démarrage réussi
	    {	
	      etat_machine = et_defaut;
	      tempoMS(0);				//remise à zéro du timer pour utilisation ultérieure
	    }

	    if (!(entrees & ve_alim))
	    {
  /*
	      flagalim = 1;
	      if (tempoMS2(tempo_coupe_alim))
	      {*/
		etat_machine = et_off;			//repasse en attente si coupure alim
		tempoMS(0);
                tempoMS2(0);
		/* flagalim = 0;
	      }
	    }
	    else
	      if (flagalim)
	      {
		tempoMS2(0);
		flagalim = false; */
	      }
	      	    
	    break;

    case et_manuel:  //****************************************************************************************************************
      sorties = vs_alim+vs_ev+vs_ok;
      if (flag2)
	{
	    manuel = true;

	    if (tempoMS(tempo_manuel))
	    {							//si pas de démarrage durant la tempo, on repasse en attente
	      etat_machine = et_attente;
	      flag2 = false;
	      manuel = false;
	    }

	    if (entrees & ve_run) {				//le moteur a démarré, passage à l'état suivant
	      etat_machine = et_run;
	      flag2 = false;
	      local = 1;
	      tempoMS(0);
              tempoMS2(0);
	    }

	    if (!(entrees & ve_alim )) {			//repasse en off si coupure alim
	      etat_machine = et_off;
	      flag2=false;
	      tempoMS(0);
              tempoMS2(0);
	      manuel = false;
	    }
	    
	    if (entrees & ve_loc)				//si la clef est tournée on actionne le démarreur
	      sorties += vs_dem;
	}
	else if(tempoMS(3E3))
	  flag2 = true;
		
	    break;
	    
    case et_pre_run:   //*************************************************************************************************************
	sorties = vs_alim+vs_ev+vs_ok;
	    
	    
	    if ((entrees & ve_alim)&&(ubat < min_chg))  	//pour éviter défaut ubat lors de la coupure alim
	    {							//si la batterie ne charge pas, allumage alarme
	      sorties += vs_alarme;
	      if (!(etat & vet_defchg))
		etat +=  vet_defchg;
	    }
	    else if (etat & vet_defchg)				//si la charge revient, on supprime le flag de défaut chg
		    etat -= vet_defchg;
	    
	    incremente_tps();					//ici le moteur tourne -> on compte le temps de fonctionnement
	    cpt_dem = 0;					//et remise à 0 du compteur d'essais de démarrages
	    if (etat & vet_defdem)
	      etat -= vet_defdem;				//et on supprime le flag défaut départ si il était présent
	      
	      
	    if (tempoMS(tempo_pre_run))				//temps de chaufffage moteur écoulé, on passe en et_run
	      etat_machine = et_run;
	    
	    if (tempoMS2(tempo_cligno))				//clignotement led OK pour signaler état
	      flag += 1;
	    
	    if (flag & 1)
	      sorties -= vs_ok;
	    	      
	    if (entrees & ve_prh)
	      if (tempoMS3(tempo_detection_defph))
	      {
		if (!(etat & vet_defph))
		  etat += vet_defph;				//passage en défaut si défaut pression huile
		  
		sorties = vs_alim+vs_alarme;
		etat_machine = et_defaut;
		Serial.println("\n\nDEFAUT PRESSION HUILE !!!\n\n");
		flag = false;
		flagrun=false;
		flag2 = false;
		local = false;
		tempoMS2(0);
		tempoMS(0);
		break;
	      }
	     
	     if (!((entrees & ve_ext )||local))
	     {
	       etat_machine = et_off;			//fin de demande externe, off
	       tempoMS(0);
	       tempoMS2(0);
	       flagext = 0;
	       break;
	     }
	     
	     if (!(entrees & ve_alim))
	     {
	       etat_machine = et_off;			//arret immediat si coupure alim
	       tempoMS(0);
	       tempoMS2(0);
	       local = false;
	       flag = false;
	       flagalim = 0;
	       break; 
	     }
	      
	    break;
	      
	    
    case et_run:  //******************************************************************************************************************
	    sorties = vs_alim+vs_ok+vs_ev+vs_out;

	    if (true)  //(!(flag||flag2)) //si on est en mode raz compteur temps, on zappe tout ce merdier
	    {
	      //if ((tps >= tps_maintenance)&&tpsm)		//en cas de dépassement délai maintenance, allumage alarme + ok
		//sorties = vs_alim+vs_ok+vs_ev+vs_alarme;


	      if ((entrees & ve_alim)&&(ubat < min_chg))
	      {							//si la batterie ne charge pas, allumage alarme seule
		sorties += vs_alarme;
		if (!(etat & vet_defchg))
		  etat +=  vet_defchg;
	      }
	      else if (etat & vet_defchg)			//si la charge revient, on supprime le flag de défaut chg
		etat -= vet_defchg;

	      incremente_tps();					//ici le moteur tourne -> on compte le temps de fonctionnement
	      cpt_dem = 0;					//et remise à 0 du compteur d'essais de démarrages
	      if (etat & vet_defdem)
		etat -= vet_defdem;				//et on supprime le flag défaut départ si il était présent

		
	      if (cpt_calage) 					//en cas de démarrage après un calage moteur,
		if (tempoMS(tempo_fin_calage))
		{						//si le moteur tourne depuis assez longtemps, on RAZ le compteur de calages
		  cpt_calage = 0;				//et on supprime le flag défaut calage si il était présent
		  if (etat & vet_defcal)
		    etat -= vet_defcal;
		}


	      if (!(entrees & ve_run))
	      {
		flagrun = true;					//si calage du moteur en cours de fonctionnement
		Serial.println("et_run : entrée dans detection calage\n");
		if(tempoMS2(tempo_detection_calage))
		  if (manuel)
		  {
		    Serial.println("tempo : manuel");
		    etat_machine = et_defaut;			//si démarrage depuis mode manuel alors passage en défaut direct
		    if (!(etat & vet_defcal))
		      etat += vet_defcal;
		    manuel = false;
		    flagrun=false;
                    tempoMS(0);
                    tempoMS2(0);
		    break;
		  }
		  else
                  {
		    Serial.println("tempo : auto");
		    etat_machine = et_calage;			//passage à l'état et_calage
		    tempoMS(0);
		    tempoMS2(0);
		    flagrun = false;
		    break;
                  }
               }   
	      else
	      {
		if (flagrun)
		{
		  tempoMS2(0);
		  flagrun = false;
                  Serial.println("sortie detect calage\n");
		}
		if (entrees & ve_prh)
		  if (tempoMS3(tempo_detection_defph))
		  {
		    if (!(etat & vet_defph))
		      etat += vet_defph;			//passage en défaut si défaut pression huile
		    sorties = vs_alim+vs_alarme;
		    etat_machine = et_defaut;
		    Serial.println("\n\nDEFAUT PRESSION HUILE !!!\n\n");
		    flag = false;
		    flagrun=false;
		    flag2 = false;
		    local = false;
		    tempoMS2(0);
		    tempoMS(0);
		    break;
		  }
	      }
		


	      if (!((entrees & ve_ext )||local))
	      {
		  etat_machine = et_post_run;			//fin de demande externe, passage au refroidissement
		  tempoMS(0);
		  tempoMS2(0);
		  flagext = 0;
		  flag = false;
		  break;
	      }

	      if (!(entrees & ve_alim))
	      {
		  etat_machine = et_off;			//arret immediat si coupure alim
		  tempoMS(0);
                  tempoMS2(0);
		  local = false;
		  flag = false;
		  flagalim = 0;
		  break; 
	      }

	    }
	  
	    

	    


	   /* //effacement du compteur de maintenance
	    if (entrees & ve_loc)
	    {							//flag sert à vérifier qu'on a pas ve_loc qui lâche puis revient avant la fin de
	      flag = true;					//tempo_valide_reset. si c'est le cas ça reset la tempo.
	      if (tempoMS(tempo_valide_reset))
	      {		
		flag = false;
		flag2 = true;
	      }
	      if (flag2)
		sorties = vs_alim+vs_ev;
	    }
	    else
	      if (flag)						//cas où local lâche avant la fin de la tempo : reset tempo
	      {
		flag = false;
		tempoMS(0);
	      }
	    
	    if (flag2)
	    {
	      if (tempoMS(tempo_montre_reset))
		flag2 = false;
	      if ( (entrees & ve_loc) && (entrees & ve_rst) )
	      {							//effacement défaut temps maintenance
		tps = 0;
		tpsm = 0;
		flag = false;
		flag2 = false;
		tempoMS(0);
		tempoMS2(0);
		etat = vet_ok;
	      }
	      
	    }
              */

  
	    break;
	    
    case et_post_run: //***************************************************************************************************************
	    sorties = vs_alim+vs_ev+vs_ok;
	    
	    if (tempoMS(tempo_post_run))
	      etat_machine = et_off;
	    
	    if (tempoMS2(tempo_cligno/2))			//clignotement led OK pour signaler état
	      flag += 1;
	    if (flag & 1)
	      sorties -= vs_ok;
	    
	   if (entrees & ve_prh)
	      if (tempoMS3(tempo_detection_defph))
	      {
		if (!(etat & vet_defph))
		  etat += vet_defph;				//passage en défaut si défaut pression huile
		sorties = vs_alim+vs_alarme;
		etat_machine = et_defaut;
		Serial.println("\n\nDEFAUT PRESSION HUILE !!!\n\n");
		flag = false;
		flagrun=false;
		flag2 = false;
		local = false;
		break;
	      }
	      
	    if (!(entrees & ve_alim))
	      etat_machine = et_off;
	    
	    if (entrees & ve_ext)
	    {
	      if(tempoMS4(tempo_valide_externe))
	      {
		etat_machine = et_run;
		tempoMS(0);
		tempoMS2(0);
	      }
	    }
	    else tempoMS4(0);
	      
	    break;
	      
	      

    case et_calage:   //**************************************************************************************************************
	    sorties = vs_alim+vs_ok+vs_alarme;

	    if (!(etat & vet_defcal))
	      etat += vet_defcal;
	    
	    if (cpt_calage > max_cpt_calage) {
	      etat_machine = et_defaut;
	      cpt_calage = 0;
	    }
	    else if (tempoMS(tempo_attente_calage)) {
		    cpt_calage++;
		    etat_machine = et_decomp;
		 }
	    break;
	    
    case et_defaut: //***************************************************************************************************************
	    sorties = vs_alim+vs_alarme;		//TODO sauver temps apparition défaut et si apparu durant run ou non
							//TODO copier le principe utilisé dans et_run pour la RAZ défaut : raz tempo si
							//la demande lâche avant validation
	    if(!flag)
	    {
	      if(etat == vet_ok )
	      {						//en cas d'entrée en défaut par erreur
		etat_machine = et_attente;
		break;
	      }


	      if(!(entrees & ve_ext))
		if (tempoMS(tempo_attente_defaut))
		  etat_machine = et_off;
	      else tempoMS(0);

	      if(!(entrees & ve_alim))
		etat_machine = et_off;

	      if (etat & vet_defubat)
		if (ubat>min_ubat_reset)
		{
		  etat -= vet_defubat;
		  etat_machine = et_attente;
		}

	      if ((entrees & ve_rst)&&(entrees & ve_loc))	//passage mode manuel seulement si défaut Vbat uniquement
		if (etat == vet_defubat)
		  {
		    etat_machine = et_manuel;
		    tempoMS(0);
		    tempoMS2(0);
		    break;
		  }
	    }
	      



	    // partie lecture/effacement défaut
	      
	      
	      
	      if ((entrees & ve_loc)&&(!flag))
	      {
		tempoMS(0);
		if (tempoMS2(tempo_montre_defaut))
		{						//si local appuyé pendant plus de tempo_montre_defaut, alors on affiche le défaut
		  flag = true;
		  tempoMS2(0);
		}
	      }
	      
	      if (flag)
	      {							
		if (!(tempoMS2(tempo_montre_defaut)))
		{
		  if (etat & vet_defdem)
		    sorties = vs_alim+vs_ok+vs_alarme;

		  if (etat & vet_defcal)
		    sorties = vs_alim+vs_ok;
		  
		  if (etat & vet_defph)		
		    sorties = vs_alim+vs_alarme;
		  
		  if (etat & vet_defubat)
		    sorties = vs_alim;

		  if ((entrees & ve_loc)&&(entrees & ve_rst))
		  {
		    tempoMS2(0);
		    if (tempoMS(tempo_valide_reset))
		    {
		      etat = 0;
		      flag = false;
		      flag2 = false;
		      tempoMS(0);
		      tempoMS2(0);
		      cpt_calage = 0;
		      cpt_dem = 0;
		      local = false;
		      etat_machine = et_attente;
		      Serial.println("reset  defaut");
		      if(sorties == vs_alim)
			sorties = vs_alim+vs_ok;
		      else
			sorties = vs_alim;
		      ecriture_sorties();
		      delay(600);
		    }
		  }
		}
		else flag = false;
	      }


	    break;
	    
	    
      default:  //*************************************************************************************************************************
	    etat_machine = et_defaut;				//si erreur interne
      
  }
}

void loop() {
  lecture_entrees();
  machine_etat();
  liaison_serie();
  ecriture_sorties();
  
}
