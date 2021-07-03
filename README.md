# AFSK_to_FSK_VFO

This repository contains the source code for "<a href="https://qrpguys.com/" target="_blank">QRPGuys</a> AFP-FSK Digital Transceiver III kit": 
https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii

as well as the mods I have originally developed for "<a href="https://qrpguys.com/" target="_blank">QRPGuys</a> DSB Digital Transceiver II Kit": 
https://qrpguys.com/qrpguys-40-30-20m-dsb-digital-transceiver-ii-kit
with "Si5351A 160m-10m VFO Kit": 
https://qrpguys.com/vfo-dsb-digital-transceiver-kit

NOTE: this is NOT an official repository from <a href="https://qrpguys.com/" target="_blank">QRPGuys</a>, just my personal repository so please DO NOT contact <a href="https://qrpguys.com/" target="_blank">QRPGuys</a> regarding the source code in this repository.

The official source code from <a href="https://qrpguys.com/" target="_blank">QRPGuys</a> is available in the product page: https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii and https://qrpguys.com/vfo-dsb-digital-transceiver-kit

# License

Since the original source code from QRPGuys is licensed under GPL v3, all the source code in this repository are also licenased under GPL v3.

# Branches

- <a href="https://github.com/kaduhi/AFSK_to_FSK_VFO/tree/main" target="_blank">main</a> - this always keeps the latest official release from <a href="https://qrpguys.com/" target="_blank">QRPGuys</a>
- <a href="https://github.com/kaduhi/AFSK_to_FSK_VFO/tree/dsb_digital_transceiver_II_mods" target="_blank">dsb_digital_transceiver_II_mods</a> - this is the mods I have developed for <a href="https://qrpguys.com/qrpguys-40-30-20m-dsb-digital-transceiver-ii-kit" target="_blank">"QRPGuys DSB Digital Transceiver II Kit"</a> with optional <a href="https://qrpguys.com/vfo-dsb-digital-transceiver-kit" target="_blank">"QRPGuys Si5351A 160m-10m VFO Kit"</a> board

# Background

In 2020, I purchased the <a href="https://qrpguys.com/qrpguys-40-30-20m-dsb-digital-transceiver-ii-kit" target="_blank">"QRPGuys DSB Digital Transceiver II Kit"</a> with optional <a href="https://qrpguys.com/vfo-dsb-digital-transceiver-kit" target="_blank">"Si5351A 160m-10m VFO Kit"</a> board after someone mentioned about the kit in <a href="https://groups.io/g/CalQRP" target="_blank">CalQRP</a> club meeting before COVID-19 pandemic.

After I built the kit, I had successfully made several FT8 contacts and I was amazed by this simple kit. However, since this is a "DSB" transceiver, it of course generates "DSB" signal, meaning both "Upper Side Band" and "Lower Side Band".

Usually the modern ham radio digital modes such as FT8 and FT4 requires a PC (to run <a href="https://physics.princeton.edu/pulsar/k1jt/wsjtx.html" target="_blank">WSJT-X</a> software) and SSB transceiver in "Upper Side Band" mode. I know it is not illegal to send the FT8 signal in DSB, but then I am using 2x the bandwidth on the valuable HF frequency.

Few weeks after <a href="https://www.instagram.com/p/CIeoMAwjRRK/" target="_blank">I modified the NanoVNA hardware and software to make it into a SDR receiver</a> and getting familiarized with the Si5351A Frequency Synthesizer chip, <a href="https://www.instagram.com/p/CJc0IOvDsft/" target="_blank">I ported</a> the SSB modulator code of <a href="https://github.com/threeme3/QCX-SSB" target="_blank">QCX-SSB</a> to <a href="https://github.com/ttrftech/CentSDR" target="_blank">CentSDR</a>. During the process, I learned that Si5351A can change the output clock frequencies 4800+ times per second!

While <a href="https://www.instagram.com/p/CJdbAUTDXa9/" target="_blank">tweaking the Class-E RF Power Amplifier</a>, I also used the PA part of the <a href="https://qrpguys.com/qrpguys-40-30-20m-dsb-digital-transceiver-ii-kit" target="_blank">"QRPGuys DSB Digital Transceiver II Kit"</a> to see <a href="https://www.instagram.com/p/CMEQAhmjWVr/" target="_blank">if I can generate more power with 5V power source</a> without DC-DC converter. Then suddenly I realized that I may be able to make this kit into a "true" FSK transmitter, just by measureing the AFSK frequency and updating the output clock frequency of Si5351A accordingly. So, <a href="https://www.instagram.com/p/CMLzBHfjkL5/" target="_blank">I have done such mods on hardware and firmware</a>, and it works!!

After <a href="https://groups.io/g/CalQRP/topic/81166950#2034" target="_blank">I posted about my mods to CalQRP group</a>, I was contacted by <a href="https://qrpguys.com/" target="_blank">QRPGuys</a> and they immediately started working on <a href="https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii" target="_blank">the new kit</a> based on my mods.

Then 6 weeks later, they have officially announced the <a href="https://qrpguys.com/qrpguys-digital-fsk-transceiver-iii" target="_blank">QRPGuys AFP-FSK Digital Transceiver III</a> kit, and start selling on May 25th, 2021. They named this technique "AFP-FSK" (Audio Frequency Processed FSK).

<a href="https://groups.io/g/kd1jvdesigns" target="_blank">KD1JV Steve Weber</a> who designed the kit <a href="https://groups.io/g/kd1jvdesigns/topic/82046651#1850" target="_blank">has already ported</a> this "AFP-FSK" technique to his old AD9850 DDS based kit and confirmed it works great.
