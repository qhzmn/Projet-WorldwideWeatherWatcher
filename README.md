Description du Dépôt : Worldwide Weather Watcher 🌍

Ce projet est développé pour l'Agence Internationale pour la Vigilance Météorologique (AIVM). Il vise à déployer des navires équipés de stations météo embarquées afin de mesurer des paramètres cruciaux pour la détection précoce de cyclones et autres catastrophes naturelles.

Matériel :
-    Arduino UNO (AVR ATmega328) → Microcontrôleur
-    Lecteur de carte SD (SPI) : stockage des données
-    Horloge RTC (I2C) : gestion de la date et de l'heure
-    LED RGB : indicateur d'état du système
-    Boutons poussoirs : interactions utilisateur
-    Pression atmosphérique, température, hygrométrie (I2C/SPI)
-    GPS (UART)
-    Luminosité (analogique)
-    
Modes de Fonctionnement :
-    Standard : Acquisition normale des données.
-    Configuration : Paramétrage du système (bouton rouge).
-    Maintenance : Accès aux données en toute sécurité (bouton rouge 5s).
-    Économique : Réduction de la consommation d'énergie (bouton vert 5s).
-    
Objectif :
Fournir une solution simple, efficace et sécurisée pour les équipages, avec une documentation utilisateur détaillée.
