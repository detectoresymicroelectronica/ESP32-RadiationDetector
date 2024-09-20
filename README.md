# Radiation Detector based on ESP32-CAM
This repository provides the files needed to program the ESP32-CAM development platform using the [ESP-IDF](https://idf.espressif.com) environment and use it as a radiation detector.

The developed code is based on the drivers published in the [ESP32 Camera Driver](https://github.com/espressif/esp32-camera) project and is intended to be used for educational purposes only by institutions with access to radioactive sources operated by qualified staff and according to local regulations. Please be aware that manipulating radiation sources or samples involves risk of irradiation and contamination.

The repository contains two individual projects located in the [Dosimetro](Dosimetro) and [Espectrometro](Espectrometro) folders. The first project consists of a dosimeter capable of estimating the dose generated by gamma radiation sources and visualizing the information obtained through a embedded web page. The second one is a spectrometer to analyze X-ray photon spectra and send the obtained histogram through the serial port, enabling its use in fluorescence analysis.

Additionally, the repository includes two folders containing supplementary information. The [Capturador](Capturador) folder contains a Python script for capturing, visualizing, and storing histograms generated by the Spectrometer. The second folder, [Extras](Extras), contains project-related information, such as the web page embedded on the Dosimeter.

## Disclaimer
This radiation detector code is provided for educational and experimental purposes only. It is not a professional-grade radiation detection system and should not be used for medical, safety, or regulatory compliance purposes.

The user assumes all risks and responsibilities associated with the use of this code in the development of a radiation detector. This includes, but is not limited to, ensuring that the device is used in a safe and appropriate manner, and that the user is aware of the potential hazards associated with radiation. 

Additionally, it is the user's responsibility to ensure that the use of this code complies with all applicable laws and regulations.

## Radiation detection using CMOS sensors
A commercial CMOS image sensor, originally designed for visible light, can also be used as an X-ray or gamma-ray detector. Although the interaction mechanism differs, the energy deposited by the incident particle generates electron-hole pairs within the sensor's volume. These charge carriers are collected by the photosensitive structures of each pixel, resulting in what is known as an 'event'. Each event can be visualized as small points in the obtained images, where the size and intensity of these points are directly related to the energy of the incident particle. Concurrently, the number of events is correlated with the quantity of photons interacting with the sensor.

Commercial cameras typically have configurations designed to enhance the quality of conventional images, which aim to reduce noise and improve scene appearance. Unfortunately, these features have a negative effect on radiation detection, such as automatic gain control or dead pixel correction. Therefore, proper camera configuration is crucial.

In these projects the camera is the OV2640 CMOS image sensor, which is able of bypassing its DSP to provide raw pixel-level data. This raw data is processed in real time by the firmware embedded in the ESP32 to determine the quantity or intensity of the events produced.

## General remarks
To prepare the ESP32-CAM for radiation detection, the OV2640 camera lens should be removed, and the sensor must be completely shielded from ambient light. Aluminum foil provides an effective and convenient method for this, fully blocking visible light while minimally attenuating X-ray and gamma-ray photons.

Although both projects share many files from the [ESP32 Camera Driver](https://github.com/espressif/esp32-camera) project, located in thier cprresponding folders, the "esp_camara.c" and "cam_hal.c" files have been adapted to meet the unique requirements of each project.

## Dosimeter
The dosimeter enables the estimation of the radiation dose produced by a high-energy gamma source through the quantification of detected events.

The measured data is visualized on an embedded web page that uses [Google Chart](https://developers.google.com/chart) for data visualization. Consequently, a connection to a device with internet access is necessary to load the required API.

Although the results obtained from this application have been validated using a calibrated Co-60 source, it is important to note that its use is restricted to educational purposes only and can not be used as a personal dosimeter.

## Spectrometer
The spectrometer can analyze the energy spectrum for photons in the 2-16keV range. The resulting histogram is transmitted through the serial port and can be displayed using the [Capturador.py](Capturador/Capturador.py) Python script.

The script's energy calibration was based on the copper emission lines. However, this calibration can be adjusted if the camera's gain setting is modified. The camera gain can be controlled using the "set_agc_gain" function within the [esp_camara.c](Espectrometro/Librerias/esp_camara.c) file.

```python
canales  = [134, 148]   #Canales de lineas de Cu
energias = [8.05, 8.91] #Energias de lineas de Cu
```

The script accepts the following command-line arguments:
- p: storage folder.
- n: file name.
- c: number of frames.
- s: serial port.

For instance, to acquire a 1000-frame histogram from the ESP32-CAM connected via COM9, measuring copper fluorescence and saving the data in the "Mediciones" folder.

```bash
python Capturador.py -p Mediciones -n Copper -c 1000 -s COM9
```

## Funding
The project was developed with funding from [Agencia Nacional de Promoción de la Investigación, el Desarrollo Tecnológico y la Innovación](https://www.argentina.gob.ar/jefatura/innovacion-ciencia-y-tecnologia/agencia) (PICT 2020-1016, PICT 2018-2886).

## Contacts
- [Damian Leonel Corzi](mailto:damian.corzi@ib.edu.ar)
- [Jose Lipovetzky](mailto:jose.lipovetzky@ib.edu.ar)
- [Mariano Gomez Berisso](mailto:iano.berisso@ib.edu.ar)
