package git1_Group7;

import java.util.Scanner;

public class Git1 {

	public static void main(String[] args) {
		int decToHex;
		boolean flag=true;
		Scanner sc= new Scanner(System.in);
		System.out.println("Please type 1 for Dec to Hex, 0 otherwise: ");
		while(flag) {
			decToHex = sc.nextInt();
			if((decToHex != 0 && decToHex !=1) ) {//validation check
				System.out.println("this number is not valid - Please enter another number: "); 
			}
			else {
				if(decToHex ==1) {//sends to the first function
					flag=false;
					int num; 
					System.out.println("Please Enter a Decimal number (int): "); 
					if(sc.hasNextInt()) {
						num = sc.nextInt();
						System.out.println("Your answer is: "+(dec2Hex (num)));
					}else {
						System.out.println("Your number is not valid - EXIT THE PROGRAM");
					}
				}else {//sends to the second function
					flag=false;
					String num;
					System.out.println("Please Enter a Hexadecimal number(String): "); 
					num = sc.next();
					System.out.println("Your answer is: "+(hex2Dec (num)));
				}
			}
		}
	}


	public static String dec2Hex(int decimalNum) {
	    String hexDigits = "0123456789ABCDEF";
	    String hexNum = "";

	    boolean isNegative = decimalNum < 0;   // check if the num is negative 

	    if (isNegative) {
	        decimalNum = -decimalNum; // change the num to positive, for transition  
	    }
	    while (decimalNum > 0) {
	        int remainder = decimalNum % 16; 
	        hexNum = hexDigits.charAt(remainder) + hexNum; // findign the right Hex num 
	        decimalNum = decimalNum / 16;
	    }
	    if (isNegative) { // if the num is negative add the (-) 
	        hexNum = "-" + hexNum;
	    }
	    return hexNum;
	}


	public static int hex2Dec(String hexNum) {
		int decimal = 0;
		String hexDigits = "0123456789ABCDEF";
		hexNum = hexNum.toUpperCase();
		for (int i = 0; i < hexNum.length(); i++) {
			char hexChar = hexNum.charAt(i);
			int hexDigit = hexDigits.indexOf(hexChar); //finds the corresponding decimal digit 
			if (hexDigit==-1) { //validation check
				System.out.println("Your number is not valid - EXIT THE PROGRAM"); 
				System.exit(1);
			}
			decimal = 16 * decimal + hexDigit; // sums up the cumulative numbers
		}
		return decimal;
	}

}//end class 


