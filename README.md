# Journal de Développement - Projet STM32

## Objectif
Développer et tester des fonctionnalités embarquées sur les microcontrôleurs STM32, en intégrant les protocoles CAN, UDS, PWM et SPI.

## Plan de Travail

### 01/08/2024
- **Objectif :** Acquérir une compréhension approfondie du protocole CAN et des outils STM32CubeIDE et PCAN-View.
- **Activités :**
  - Intégration du protocole CAN dans des projets embarqués.
  - Développement et débogage sur STM32F407.
  - Test de transmission CAN avec PCAN-View.

### 02/08/2024
- **Objectif :** Envoyer des messages CAN avec différents IDs.
- **Activités :**
  - Envoi de messages CAN avec ID standard (11 bits) et étendu (29 bits).
  - Développement d'un code pour envoyer une trame CAN contenant le résultat de conversion de l'ADC, configuré en mode DMA.

### 05/08/2024
- **Objectif :** Modifier le baudrate du CAN.
- **Activités :**
  - Création d'un service pour changer le baudrate du CAN.
  - Test du service avec le logiciel PCAN.

### 06/08/2024
- **Objectif :** Étudier les filtres CAN et mettre en place la réception de trames.
- **Activités :**
  - Étude des concepts de filtres CAN.
  - Implémentation de la réception d'une trame CAN.

### 07/08/2024
- **Objectif :** Développer un service de filtrage CAN.
- **Activités :**
  - Filtrage des messages CAN en fonction de leur ID (standard ou étendu).
  - Développement d'un code pour recevoir 4 messages CAN distincts (2 IDs standard et 2 IDs étendus).

### 08/08/2024
- **Objectif :** Intégrer le protocole UDS et gérer les GPIO.
- **Activités :**
  - Recherche sur le protocole UDS.
  - Développement d'un service GPIO pour contrôler les LEDs via la réception de trames CAN.
  - Envoi de trames de validation ou d'erreur en fonction du diagnostic UDS.

### 09/08/2024
- **Objectif :** Mettre en place un service ADC via CAN.
- **Activités :**
  - Développement d'un service ADC avec API pour démarrer, récupérer les résultats de conversion et arrêter l'ADC via CAN.

### 12/08/2024 - 13/08/2024
- **Objectif :** Développer un service pour générer des signaux PWM.
- **Activités :**
  - Recherche sur le signal PWM configuré en mode sortie.
  - Développement d'un service pour générer des signaux PWM avec deux API : SET PWM et GET PWM.
  - Utilisation d'un oscilloscope pour vérifier la précision du signal PWM.

### 14/08/2024 - 16/08/2024
- **Objectif :** Gérer les signaux PWM en mode entrée.
- **Activités :**
  - Recherche sur le signal PWM en mode entrée (PWM input et input capture).
  - Développement d'un service pour gérer le signal PWM en mode entrée et visualisation sur l'oscilloscope.

### 19/08/2024
- **Objectif :** Organiser le code en fichiers distincts.
- **Activités :**
  - Décomposition des services GPIO, ADC, PWM (entrée et sortie) en fichiers d'en-tête (.h) et fichiers source (.c).

### 20/08/2024
- **Objectif :** Réimplémenter le projet sur la carte STM32F429I.
- **Activités :**
  - Réimplémentation complète du projet sur la nouvelle carte STM32F429I.

### 21/08/2024 - 23/08/2024
- **Objectif :** Assurer la communication CAN entre trois nœuds.
- **Activités :**
  - Mise en place d'une communication CAN entre PCAN, STM32F407 et STM32F429I.
  - Développement et validation des services pour le diagnostic et la communication entre les nœuds.

### 26/08/2024
- **Objectif :** Recherche sur le protocole SPI.
- **Activités :**
  - Exploration des modes de fonctionnement, de synchronisation et de gestion des données du protocole SPI.

### 27/08/2024 - 29/08/2024
- **Objectif :** Développer des programmes SPI.
- **Activités :**
  - Développement de programmes SPI pour les cartes STM32F407 (maître et esclave) pour transmettre des résultats ADC via des messages de débogage.

### 30/08/2024
- **Objectif :** Validation et gestion de version du projet.
- **Activités :**
  - Validation du travail effectué.
  - Importation du projet complet sur GitHub pour gestion de version et partage du code.

## Notes
- Chaque étape est conçue pour construire progressivement les compétences nécessaires et assurer l'intégration réussie des différents protocoles et services.
