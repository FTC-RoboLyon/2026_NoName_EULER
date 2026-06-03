// C quoi Main (autre projet ?) rien a faire là en plus peut peut etre causer erreurs puisque java cherche toujours un Main pour commencer. à supprimer


import java.util.Scanner;
public class Main{

 public static void main(String args[]){
    Scanner input = new Scanner(System.in);
    int température = input.nextInt(); 
    if (température < 0)
        System.out.println("froid");
  }
}