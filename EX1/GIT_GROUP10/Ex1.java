import java.util.Scanner;

public class Ex1 {

	public static void main(String[] args) {
		Scanner scanner = new Scanner(System.in);
		boolean baseConversion = true; // continue to convert

		while(baseConversion) { // the user wants to continue
			System.out.println("Please choose a base conversion:");
			System.out.println("1. 10 to 16");
			System.out.println("2. 16 to 10");
			String option = scanner.next(); // user input

			while ((!option.equals("1")) && (!option.equals("2"))) { // check input validation
				System.out.println("erorr!!, choose 1 or 2");
				option = scanner.next();
			}

			if (option.equals("1")) { // convert from 10 to 16 
				System.out.println("the HexaDecimal number is: " +  option1(option));
			}
			if (option.equals("2")) { // convert from 16 to 10
				System.out.println("the Decimal number is: " +  option2(option));  
			}

			System.out.println("Do you want to continue?");
			String continueConv = scanner.next(); // user answer
			while(!(continueConv.equalsIgnoreCase("yes") || continueConv.equalsIgnoreCase("no") )) { // check answer validation
				System.out.println("please enter valid answer");
				continueConv = scanner.next();
			}

			baseConversion = continueConv.equalsIgnoreCase("yes"); // update the user answer
		}

		System.out.println("GoodBye"); // end the calculate
	}

	public static String option1 (String decNum) // convert from 10 to 16
	{
		Scanner scanner = new Scanner(System.in);
		System.out.println("please enter a Dec number:");
		String inputDec = scanner.next();
		while(!checkDec(inputDec)) { // check if the number is Decimal
			inputDec = scanner.next();
		}

		return calc_10to16(inputDec); // calculate 
	}


	public static String calc_10to16(String inputDec) { // return the HexaDecimal Number
		int inputDec1 = Integer.parseInt(inputDec); // convert string to int
		String hexaDec= "";
		while(inputDec1!= 0) {
			int remainder = inputDec1 % 16; 
			hexaDec = hexDigit(remainder) + hexaDec; // add the reminder to the list
			inputDec1= inputDec1/16;

		}

		return hexaDec;
	}

	private static char hexDigit(int num) { // return letter instead of number(num>=10)
		if (num < 10) {
			return (char) (num + '0'); 
		} else {
			return (char) (num - 10 + 'A'); 
		}
	}

	public static boolean checkDec(String inputDec) { // check if the number is Decimal
		for (int i=0; i<inputDec.length();i++) {
			if ((inputDec.charAt(i)<'0') || (inputDec.charAt(i)>'9')) {
				System.out.println("Enter a valid Decimal Number");
				return false;
			}
		}
		return true;
	}


	public static int option2 (String hexNum) // convert from 16 to 10
	{
		Scanner scanner = new Scanner(System.in);
		System.out.println("please enter a hex number:");
		String inputHex = scanner.next();
		while(!checkHex(inputHex)) { // check if the number is HexaDecimal
			inputHex = scanner.next();
		}

		return calc_16to10(inputHex); // calculate
	}

	public static int calc_16to10(String inputHex) { // return the Decimal Number
		int decimal = 0;
		int power = inputHex.length() - 1; // the power according to the position of the digit

		for (int i = 0; i < inputHex.length(); i++) {
			char hexDigit = inputHex.charAt(i);
			int digit = 0;
              
			//  return number instead of letter
			if (hexDigit >= '0' && hexDigit <= '9') {
				digit = hexDigit - '0';
			} else if (hexDigit >= 'A' && hexDigit <= 'F') {
				digit = hexDigit - 'A' + 10;
			} else if (hexDigit >= 'a' && hexDigit <= 'f') {
				digit = hexDigit - 'a' + 10;
			}

			decimal += digit * Math.pow(16, power);
			power--;
		}

		return decimal;
	}

	public static boolean checkHex(String inputHex) { // check if the number is HexaDecimal
		for (int i=0; i<inputHex.length();i++) {
			if (!((inputHex.charAt(i)>='0' && inputHex.charAt(i)<='9') || 
					(inputHex.charAt(i)>='A' && inputHex.charAt(i)<='F') || 
					(inputHex.charAt(i)>='a' && inputHex.charAt(i)<='f'))) {
				System.out.println("Enter a valid HexaDecimal Number");
				return false;
			}
		}
		return true;
	}



}
