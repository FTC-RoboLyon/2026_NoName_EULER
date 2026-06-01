import java.util.Scanner;

public class Main{

 public static void main(String args[]){
    Scanner input = new Scanner(System.in);
    int température = input.nextInt(); 
    if (température < 0)
        System.out.println("froid");
  }
}