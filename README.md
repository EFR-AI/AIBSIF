<h1 align="center">AIBSIF</h1>

<div align="center">
  🚀 <strong> AI-Based Sensor Information Fusion </strong>
</div>
<div align="center">
  End of study project of a group of 6 persons at EFREI Paris in the framework of PERSEUS
(Student Project of European Space Research University and Scientific) organized by the Direction of the Launchers of the National Center of Space Studies (CNES)
</div>

<br />

<div align="center">
  <!-- license -->
  <a href="https://www.mozilla.org/en-US/MPL/2.0/">
    <img src="https://img.shields.io/github/license/EFR-AI/AIBSIF?style=for-the-badge"
      alt="license" />
  </a>
  <!-- Maintenance -->
    <img src="https://img.shields.io/maintenance/yes/2022?style=for-the-badge"
      alt="Maintenance" />
  <!-- Size -->
    <img src="https://img.shields.io/github/repo-size/EFR-AI/AIBSIF?style=for-the-badge"
      alt="size" />
  <!-- Last Commit -->
  <a href="https://github.com/EFR-AI/AIBSIF/commit/main">
    <img src="https://img.shields.io/github/last-commit/EFR-AI/AIBSIF?style=for-the-badge"
      alt="Last Commit" />
  </a>
  <!-- Activity -->
  <a href="https://github.com/EFR-AI/AIBSIF/graphs/commit-activity">
    <img src="https://img.shields.io/github/commit-activity/w/EFR-AI/AIBSIF?style=for-the-badge"
      alt="Activity" />
  </a>
  <!-- PR -->
  <!--  <img src="https://img.shields.io/github/status/contexts/pulls/EFR-AI/AIBSIF/0?style=for-the-badge"-->
  <!--    alt="pulls" />-->
</div>

<div align="center">
  <sub>Project made with ❤︎ by
  <a href="https://github.com/CleaverEFREI">Louis Gailhac</a> ,
  <a href="tbd">Louis Dumontet</a> ,
  <a href="https://github.com/Duramann">Théo Dura</a> ,
  <a href="https://github.com/Matthieu-Ecc">Matthieu Eccher</a> ,
  <a href="https://github.com/numan-sahnou">Numan Sahnou</a> ,
  <a href="tbd">Vitor Serena</a> and 
  <a href="https://github.com/EFR-AI/AIBSIF/graphs/contributors">
    contributors
  </a>
</div>

## Organization:
<div align="center">
<a href="https://www.perseusproject.com/" target="_blank"><img src="https://perseus.cnes.fr/sites/default/files/styles/medium/public/drupal/201907/image/bpc_perseus_logo_allege.jpg" width="133" height="100" ></a>
<a href="https://www.cnes.fr/en" target="_blank"><img src="https://cnes.fr/sites/default/files/drupal/201707/image/is_logo_2017_logo_carre_bleu.jpg" width="105" height="100"></a>
<a href="https://www.efrei.fr/" target="_blank"><img src="https://www.efrei.fr/wp-content/uploads/2019/06/Logo-Efrei-2017-Fr-Web.png" width="270" height="100"></a>
</div>
  
## Table of Contents
- [Team](#team)
- [Data generation](#team)
- [Working with our model](#team)
  
## Team
<div align="center">
<a href="https://github.com/CleaverEFREI" target="_blank"><img src="https://cdn.discordapp.com/avatars/263637198023163914/bcef7e6064e490377b526cc4edda37d6.png?size=100"></a>
<a href="tbd" target="_blank"><img src="https://cdn.discordapp.com/icons/916284262855106561/b04b5d8a302498a461bc7198664b710a.png?size=100"></a>
<a href="https://github.com/Duramann" target="_blank"><img src="https://media.discordapp.net/attachments/916311623973609522/916312189428723752/unknown.png"></a>
<a href="https://github.com/Matthieu-Ecc" target="_blank"><img src="https://cdn.discordapp.com/avatars/366621325143441408/af9e1f5d98b863017a76a042a1fc9258.png?size=100"></a>
<a href="tbd" target="_blank"><img src="https://cdn.discordapp.com/avatars/152191486820089856/66950e226fb9f169c64b157c49eacb31.png?size=100"></a>
<a href="https://github.com/numan-sahnou" target="_blank"><img src="https://cdn.discordapp.com/icons/916284262855106561/b04b5d8a302498a461bc7198664b710a.png?size=100"></a>
</div>
  
## Data generation
  
In order to produce some data we used the flight rocket simulator developed by a PERSEUS team of Central Lille. You can find the code in AIBSIF/simulateur/Kaos-master/

We focus on the main.py file. In main the my_rocket objet was called from the sera3.py file that you can find in AIBSIF/simulateur/Kaos-master/rockets . The sera3.py containt an accurate object representing the SERA3 rocket with exacte physical parameters, but also inertial measurement unit which are configurable with "noise_power" and "erreur_de_justesse".
Previously in the first version of the code the obejct sera3 was called only once leading to variation null in our data generation. This is why we took the entire code of sera3 and put in directly in the main.py file. This way we generate slightly different data from each other.
we randomized this parameter:

    accelerometres_list=[Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=random.random()/100,erreur_de_justesse=random.randint(0,10)/100),Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=random.random()/100, erreur_de_justesse=random.randint(0,10)/100),Accelerometre([2.09,0.0,0.0],phi=0.0, theta=0.0, psi=0.0, incertitude_phi=0.0, incertitude_theta=0.0, incertitude_psi=0.0, noise_power=random.random()/100, erreur_de_justesse=random.randint(0,10)/100)],

    gyroscopes_list = [Gyroscope('zyx', noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100),Gyroscope('zyx', noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100),Gyroscope('zyx', noise_power=random.random()/10, erreur_de_justesse=random.randint(0,10)/100)],
        
    puissance_vent = random.choice([0,1,2])
    longeur_rampe = random.choice([7,8,9,10,11,12,13,14,15])
    latitude  =  random.randint(50,70)
    longitude =  random.randint(10,30)


If you want to generate new data you can keep this code or modifie parameter, you can also change rocket model but keep in mind that our machine leanrning model needs 3 IMU of each type. then execute main.py file. the code will produce a N number of data set according to the value of the loop:

    for name in tqdm(range(20)): #here the value is 20

Data produced with no error will be register in folder resultats_3IMU and data produced with randomized error will be register in folder sensors_data_3IMU

## Working with our model