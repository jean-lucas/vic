package vicSim;

import java.util.Random;

public class Car implements Runnable {
	
	
	private int comingFrom;
	private int goingTo;
	private final int ID;
	private int priority;

	
	public Car(int cf, int gt, final int id) {
		this.comingFrom = cf;
		this.goingTo = gt;
		this.ID = id;
		this.priority = -1;
	}
	
	
	public String sendRequest() {
		String signal = "" + this.ID + "_" + this.comingFrom + "_" + this.goingTo;
		return signal;
	}
	
	public int getCarId() {
		return this.ID;
	}
	
	public int getCf() {
		return this.comingFrom;
	}
	
	public int getGt() {
		return this.goingTo;
	}
	
	public void setPri(int pri) {
		this.priority = pri;
	}
	
	public int getPriority() {
		return this.priority;
	}
	
	public String prettyPrintStatus() {
		
		String comingDirection = getDirectionString(this.getCf());
		String goingDirection  = getDirectionString(this.getGt());		
		String s = String.format("%3d %10d %20s %20s", this.getCarId(), this.getPriority(), comingDirection, goingDirection);
		
		return s;
	}

	private String getDirectionString(int val) {
		String dir;
		switch(val) {
			case 1:
				dir = "North";
				break;
			case -1:
				dir = "South";
				break;
			case 2:
				dir = "East";
				break;
			default:
				dir =  "West";
					break;
		}
			return dir;
	}	
	
	
	private void generateRandomInfo() {
		
		Random rand = new Random();
		int r1,r2,sign1,sign2;
		r1 = 1 + rand.nextInt(2);
		sign1 = rand.nextInt(2);
		r2 = 1 + rand.nextInt(2);
		sign2 = rand.nextInt(2);
		
		r1 = (sign1 == 0) ? -1*r1 : r1;
		r2 = (sign2 == 0) ? -1*r2 : r2;
		
		this.goingTo = r1;
		this.comingFrom = r2;
		
	}
	
	
	@Override
	public void run() {
			
			// Generate a new event for this car, and sleep
			// for a random time between 0-499 ms
			this.generateRandomInfo();
			Random seed = new Random();
			try {
				Thread.sleep(seed.nextInt(500));
			} 
			catch (InterruptedException e) {}
	}
}



















