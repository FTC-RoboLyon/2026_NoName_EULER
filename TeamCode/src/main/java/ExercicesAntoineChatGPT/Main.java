package ExercicesAntoineChatGPT;

import java.util.Scanner;

public class Main {
    public static void main(String[] args) {

        Scanner scanner = new Scanner(System.in);

        VisionConfig config = new VisionConfig(5.1, 0.5);

        while (true) {
            config.afficher();

            System.out.print("Nouvelle hauteur : ");
            double nouvelleHauteur = scanner.nextDouble();

            System.out.print("Nouvelle largeur : ");
            double nouvelleLargeur = scanner.nextDouble();

            config.setHauteur(nouvelleHauteur);
            config.setLargeur(nouvelleLargeur);

            System.out.println("Valeurs mises à jour.\n");
        }
    }
}
