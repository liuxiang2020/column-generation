package com.vrptw.timer;



public class TestTimer extends Thread {  
	  public static void main(String[] args)  throws InterruptedException  {
	
			Timer watch = new Timer();
			watch.start();
			
			for(int i = 1; i <= 10; i++) {
				Thread.sleep(100);
				System.out.println(watch.getMillisSecondString());

			}
	    
			watch.stop();
			System.out.println(watch.toString());
	  }
}
