MINIPROJET — Application de reconnaissance simple avec camera 

Cette application permet de faire une reconnaissance simple entre deux classes appelées P1 et P2 à partir de l’image caméra.

La caméra affiche une image en direct sur l’écran. L’utilisateur peut enregistrer plusieurs images pour P1 et P2, puis lancer une reconnaissance avec le bouton TEST.

Interface
---------

L’écran affiche quatre boutons tactiles :

- P1 + : ajouter une image d’apprentissage pour P1.
- P2 + : ajouter une image d’apprentissage pour P2.
- TEST : tester l’image courante.
- RESET : réinitialiser l’application.

Utilisation étape par étape
---------------------------

1. Ajouter les images de P1

Placez la première personne ou le premier objet devant la caméra.

Appuyez sur P1 +.

À chaque appui, l’image courante est ajoutée à la classe P1.

Après l’appui, un filtre de Sobel vert apparaît pendant environ 2 secondes. Cela confirme que l’image a bien été prise en compte.

Il faut ajouter 6 images pour P1.

Le compteur affiché à l’écran indique la progression :

P1:1/6
P1:2/6
...
P1:6/6

2. Ajouter les images de P2

Placez la deuxième personne ou le deuxième objet devant la caméra.

Appuyez sur P2 +.

À chaque appui, l’image courante est ajoutée à la classe P2.

Après l’appui, un filtre de Sobel bleu apparaît pendant environ 2 secondes.

Il faut ajouter 6 images pour P2.

Le compteur affiché à l’écran indique la progression :

P2:1/6
P2:2/6
...
P2:6/6

3. Passage automatique en mode reconnaissance

Lorsque les 6 images de P1 et les 6 images de P2 ont été ajoutées, l’application passe automatiquement en mode reconnaissance.

L’écran affiche alors :

RECO PRETE

À partir de ce moment, les boutons P1 + et P2 + ne servent plus à ajouter des images.

4. Lancer une reconnaissance

Placez une personne ou un objet devant la caméra.

Appuyez sur TEST.

L’application compare l’image courante avec les images apprises de P1 et P2.

Le résultat affiché sera :

RESULTAT : PERSONNE 1

ou :

RESULTAT : PERSONNE 2

L’écran affiche aussi deux distances :

d1 = distance avec P1
d2 = distance avec P2

La plus petite distance correspond à la classe reconnue.

5. Réinitialiser l’application

Pour recommencer l’apprentissage depuis zéro, appuyez sur RESET.

Cela efface :
- les images apprises de P1 ;
- les images apprises de P2 ;
- le dernier résultat ;
- les distances affichées.

L’application revient ensuite en mode apprentissage.

Fonctionnement 
--------------

L’application ne compare pas directement toute l’image caméra.

Elle prend une zone centrale de l’image, puis la réduit en une petite image de 24 x 24 pixels.

Chaque image est convertie en niveaux de gris puis normalisée.

Pendant l’apprentissage, l’application calcule une image moyenne pour P1 et une image moyenne pour P2.

Pendant le test, elle compare l’image courante avec ces deux moyennes.

Le résultat est la classe dont l’image moyenne est la plus proche.

États de l’application
----------------------

Démarrage
   ↓
Mode apprentissage
   ↓
Ajout de 6 images P1
   ↓
Ajout de 6 images P2
   ↓
Mode reconnaissance
   ↓
Appui sur TEST
   ↓
Affichage du résultat

À tout moment, le bouton RESET permet de revenir au début.
