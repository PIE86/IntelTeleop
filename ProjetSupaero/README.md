1. Ici, je n'ai fait qu'un seul pas de calcul de control optimal et un pas de simulation.
2. Le lien entre les deux parties est fait par les fichiers de données
3. Il reste encore quelques modifications pour finaliser le travail:
    (1) Il faut encore fermer la boucle pour que le changement d'état du système est renouvelé dans le même fichier
        (maintenant, l'état avant et celui après l'évolution du système sont dans deux fichiers différents).
    (2) Il faut encore tester avec une période de temps plus longue/plus de nombre de pas de calcul de control 
        et de simulation.
    (3) Le pas de calcul pour la synthèse de loi de contrôle est configuré identique avec celui de la simulation, 
        mais il faut encore rassurer.
    (4) Pour aller un peu plus loin, il vaut mieux de réaliser ce processus en utilisant la programmation orientée objet
        (jusqu'à maintenant, pour d'abord réaliser la fonction, je n'ai utilisé que la programmation orienté procédure),
        ansi, on pourra éviter d'utiliser les fichiers de données.
    (5) Pour les codes, ils sont un peu désordonnés, il me reste encore à faire des traitements dessus.
4. Pour l'aspect modélisation, j'ai deux idées:
    (1) Soit on fait la synthèse de loi de commande avec un modèle plus simple, et on fait la simulation avec le modèle offert
    (2) Soit on fait la synthèse de loi de commande avec le modèle actuel, et on fait la simulation de validation avec un
        modèle encore plus complexe, avec l'introduction des bruits de mesures ou des perturbations par exemple.
