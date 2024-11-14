import java.util.Scanner;

public class hexadecimalDecimalCalc {

	//convert from decimal to hexadecimal
	public static String decimalToHex(double decimal) 
	{
	    //The integer part
	    int integerPart = (int) decimal;
	    double fractionalPart = decimal - integerPart;

	    // Convert the integer part to hexadecimal
	    String hex = "";
	    char[] hexChars = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

	    while (integerPart > 0) {
	        int remainder = integerPart % 16;
	        hex = hexChars[remainder] + hex;
	        integerPart /= 16;
	    }

	    if (hex.isEmpty()) {
	        hex = "0"; // For cases where the integer part is 0
	    }

	    //The fractional part
	    if (fractionalPart > 0) {
	        hex += ".";
	        for (int i = 0; i < 8; i++) {  // Limit to 8 digits after the decimal point
	            fractionalPart *= 16;
	            int fractionalHexDigit = (int) fractionalPart;
	            hex += hexChars[fractionalHexDigit];
	            fractionalPart -= fractionalHexDigit;

	            if (fractionalPart == 0) break;
	        }
	    }

	    return hex;
	}


	// Convert from hexadecimal to decimal
	public static double hexToDecimal(String hex) 
	{
	    //Split the hexadecimal string
	    String[] parts = hex.split("\\.");
	    int integerPart = 0;

	    //Convert the integer part from hexadecimal to decimal
	    for (int i = 0; i < parts[0].length(); i++) {
	        char hexChar = parts[0].charAt(i);
	        integerPart = integerPart * 16 + hexCharToDecimal(hexChar);
	    }

	    double fractionalPart = 0.0;
	    // Check if there is a fractional part
	    if (parts.length > 1) {
	        double base = 1.0 / 16;
	        //Convert the fractional part 
	        for (int i = 0; i < parts[1].length(); i++) {
	            char hexChar = parts[1].charAt(i);
	            fractionalPart += hexCharToDecimal(hexChar) * base;
	            base /= 16;  //Update base for the next fractional place
	        }
	    }

	    // Return the sum of the integer and fractional parts in decimal
	    return integerPart + fractionalPart;
	}


	private static int hexCharToDecimal(char hexChar) {
	    //Check if hexChar is a digit between '0' and '9'
	    if (hexChar >= '0' && hexChar <= '9') {
	        return hexChar - '0';  //Convert char '0'-'9' to int 0-9
	    } 
	    //Check if hexChar is a letter between 'A' and 'F'
	    else if (hexChar >= 'A' && hexChar <= 'F') {
	        return hexChar - 'A' + 10;  //Convert char 'A'-'F' to int 10-15
	    } 
	    //Check if hexChar is a letter between 'a' and 'f'
	    else if (hexChar >= 'a' && hexChar <= 'f') {
	        return hexChar - 'a' + 10;  //Convert char 'a'-'f' to int 10-15
	    } 
	    // Throw an exception if hexChar is not a valid hexadecimal character
	    else {
	        throw new IllegalArgumentException("Invalid hexadecimal character");
	    }
	}


	public static void main(String[] args) {
	    Scanner scanner = new Scanner(System.in);
	    System.out.println("Base Conversion Calculator: Decimal-Hexadecimal");

	    while (true) {
	        System.out.println("Choose an option:");
	        System.out.println("1. Decimal to Hexadecimal");
	        System.out.println("2. Hexadecimal to Decimal");
	        System.out.println("3. Exit");

	        int choice;
	        //Check if the user has entered an integer for the menu choice
	        if (scanner.hasNextInt()) {
	            choice = scanner.nextInt();
	            scanner.nextLine(); 

	            switch (choice) {
	                case 1:
	                    System.out.print("Enter a decimal number: ");
	                    //Check if the input is a valid decimal 
	                    if (scanner.hasNextDouble()) {
	                        double decimal = scanner.nextDouble();
	                        //Call the decimalToHex function 
	                        System.out.println("Hexadecimal conversion: " + decimalToHex(decimal));
	                    } else {
	                        //Display error message if input is invalid
	                        System.out.println("Invalid input. Please enter a valid decimal number.");
	                        scanner.next(); 
	                    }
	                    break;

	                case 2:
	                    System.out.print("Enter a hexadecimal number: ");
	                    String hex = scanner.nextLine();
	                    try {
	                        //Call the hexToDecimal function 
	                        System.out.println("Decimal conversion: " + hexToDecimal(hex));
	                    } catch (IllegalArgumentException e) {
	                        //Display errors in hexadecimal input
	                    	    System.out.println("Invalid input. Please enter a valid hexadecimal number.");
	                    	
	                    }
	                    break;

	                case 3:
	                    //Exit option: Close the scanner and end the program
	                    System.out.println("Bye Bye");
	                    scanner.close();
	                    return;

	                default:
	                    //Display message if an invalid choice is entered
	                    System.out.println("Invalid choice, please choose again.");
	            }
	        } else {
	            //Display error message if the input is not a valid integer
	            System.out.println("Invalid choice, please choose a number from the options.");
	            scanner.next(); 
	        }
	    }
	}
}
