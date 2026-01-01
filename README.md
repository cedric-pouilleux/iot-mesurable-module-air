# IoT Mesurable - Air Test Module

Module ESP32 minimal pour tester le système IoT Mesurable (backend + frontend).

## 📋 Prérequis

- **PlatformIO** (VSCode extension ou CLI)
- **ESP32-S2 Mini** (ou autre ESP32)

## 🚀 Installation

### 1. Compiler et uploader

```bash
cd iot-mesurable-module-air
pio run -t upload
```

### 2. Premier démarrage - Configuration WiFi

Au premier boot, l'ESP32 crée un point d'accès WiFi :
- **SSID** : `IoT-air-test-01`
- Connectez-vous et allez sur `192.168.4.1`
- Configurez votre WiFi et l'adresse du broker MQTT

### 3. Configuration MQTT

Dans le portail WiFiManager, entrez :
- **MQTT Broker** : L'IP de votre PC (ex: `192.168.1.100`)
- **MQTT Port** : `1883`

## 📊 Ce que fait ce module

- Publie toutes les **5 secondes** :
  - `temperature` : 20-25°C (simulé)
  - `humidity` : 50-60% (simulé)
  - `co2` : 400-550 ppm (simulé)
  
- La LED clignote à chaque publication

## 🔍 Monitoring

```bash
pio device monitor
```

Sortie attendue :
```
[BOOT] Starting WiFi connection...
[OK] Connected to WiFi & MQTT!
[OK] Sensors registered:
     - temperature (°C)
     - humidity (%)
     - co2 (ppm)
[READY] Publishing every 5 seconds...

[PUBLISH] temp=22.5°C  hum=57.2%  co2=485ppm
[PUBLISH] temp=21.8°C  hum=54.9%  co2=512ppm
```

## ⚙️ Personnalisation

Dans `src/main.cpp` :
- Changer `"air-test-01"` pour un autre module ID
- Modifier `PUBLISH_INTERVAL` pour changer la fréquence
- Ajouter d'autres capteurs simulés
