package vicSim;

public class Car {
	
	
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
	
	
//	@Override
//	void run() {
//		for (int i = 0; i < 5; i++) {
//			this.sendRequest();
//		}
//	}
}



















