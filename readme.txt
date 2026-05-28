MINIPROJET 442

Application de reconnaissance simple sur STM32 avec caméra, écran LCD et interface tactile. Le système apprend deux classes, P1 et P2, puis classe une nouvelle image avec le bouton TEST. 

Utilisation de la caméra :
La caméra sert à capturer l’image courante affichée à l’écran. En mode apprentissage, on place successivement la personne ou l’objet devant la caméra puis on appuie sur P1+ ou P2+ pour enregistrer l’image dans la bonne classe. Après 6 images pour P1 et 6 images pour P2, l’application passe en mode reconnaissance. En mode test, on place la personne ou l’objet devant la caméra puis on appuie sur TEST pour obtenir le résultat. 

Commandes :
- P1+ : ajoute une image à P1
- P2+ : ajoute une image à P2
- TEST : lance la reconnaissance
- RESET : remet l’application à zéro 

Principe :
L’application extrait la zone centrale de l’image, la réduit en 24x24 pixels, la convertit en niveaux de gris, puis compare l’image courante aux modèles appris pour P1 et P2. 
