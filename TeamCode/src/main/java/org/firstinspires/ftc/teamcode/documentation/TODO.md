TODO :
- gerer le stop à la fin de l'opMode
- renommer Driver en TankDriver
- extraire les magic number (exemple: setPower(0.7)) dans les constantes
- ajouter un fichier AGENTS.md pour que gemini comprenne bien le projet (demandez à gemini sur le
  net comment faire)
- creer une classe Robot qui encapsulent tous les systemes afin de simplifier l'OpMode
- passer sur un OpMode iteratif plutot que LinearOpMode (conseillé en teleop)
- ajouter Gyro, Vision
- tester sur le robot
- enfin, tenter un mode autonome (avec plusieurs position de démarrage/imu/cam....)

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
