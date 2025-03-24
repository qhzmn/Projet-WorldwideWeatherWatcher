# Worldwide Weather Watcher 🌍

## Description du Dépôt

Ce projet est développé pour l'Agence Internationale pour la Vigilance Météorologique (AIVM). Il vise à déployer des navires équipés de stations météo embarquées afin de mesurer des paramètres cruciaux pour la détection précoce de cyclones et autres catastrophes naturelles.

## Matériel

- **Arduino UNO (AVR ATmega328)** : Microcontrôleur
- **Lecteur de carte SD (SPI)** : Stockage des données
- **Horloge RTC (I2C)** : Gestion de la date et de l'heure
- **LED RGB** : Indicateur d'état du système
- **Boutons poussoirs** : Interactions utilisateur
- **Capteurs de pression atmosphérique, température, hygrométrie (I2C/SPI)**
- **GPS (UART)**
- **Capteur de luminosité (analogique)**

## Modes de Fonctionnement

- **Standard** (LED verte continue) : Acquisition normale des données.
- **Configuration** (LED jaune continue) : Paramétrage du système (bouton rouge).
- **Maintenance** (LED orange continue) : Accès aux données en toute sécurité (bouton rouge 5s).
- **Économique** LED bleue continue) : Réduction de la consommation d'énergie (bouton vert 5s).

## Objectif

Fournir une solution simple, efficace et sécurisée pour les équipages, avec une documentation utilisateur détaillée.

## Table des Matières

1. [Description du Dépôt](#description-du-dépôt)
2. [Matériel](#matériel)
3. [Modes de Fonctionnement](#modes-de-fonctionnement)
4. [Objectif](#objectif)

---

N'hésitez pas à personnaliser davantage en fonction de vos besoins spécifiques !
