package ExercicesAntoineChatGPT;

public class VisionConfig {

    private double hauteur;
    private double largeur;

    public VisionConfig(double hauteur, double largeur) {
        this.hauteur = hauteur;
        this.largeur = largeur;
    }

    public void afficher() {
        System.out.println("Valeurs actuelles :");
        System.out.println("Hauteur = " + hauteur);
        System.out.println("Largeur = " + largeur);
    }

    public void setHauteur(double hauteur) {
        this.hauteur = hauteur;
    }

    public void setLargeur(double largeur) {
        this.largeur = largeur;
    }
}

