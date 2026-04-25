TODO :

- ajouter un fichier AGENTS.md pour que gemini comprenne bien le projet (demandez à gemini sur le
  net comment faire)
- ajouter Vision
- ajouter des Steps associées à la vision/odometry
- suppr branch raf
- suppr Constantes (tout rappatrier dans les classes necessaires)
- tester sur le robot

# Git / Github

Le projet actuel contient deja 2 remotes github (`git remote -vv`)

- origin = le repo github robolyon
- upstream = le repo ftc pour le sdk

- Pour passé d'une branche à une autre :

- depuis l'IDE (android studio), en haut (marqué `nath`), on deroule le menu, et on voit `seb` puis
  checkout et inversement
- en ligne de commande
    - `git checkout seb` ou `git checkout nath` (si les branches existent deja)
    - `git checkout -b <nom branche locale voulue> <remote>/<nom branche distance>`
- ATTENTION : il faut avoir un repertoire de travail sans modif pour pouvoir changé de branche (cf
  la vue commit ou `git status`)

# Branche `main`

Toujours etre en sync avec `main`

- `git co main` + `git fetch` + `git pull`
- creer une nouvelle branche de travail depuis `main` : `git checkout -b <nom> origin/main`
  fait une class odometrie qui renvoie la position du robot par les encodeurs de deux nouveaux
  moteurs décentralisés et espacé d'une distance x. Ils sont reiés a deux roues mortes qui sont dans
  le méme sens, sens des rous du driver
- ajouter un fichier AGENTS.md pour que gemini comprenne bien le projet avec ce que tu as compris du
  contexte du programme (objectif faire un robot fonctionnel pour jouer au jeu ftc Decode)
