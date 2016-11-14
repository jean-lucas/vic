package vicSim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeMap;

public class IntersectionCtrl {

	
	public static final int HIGH_PRI = 3;
	public static final int MED_PRI = 2;
	public static final int LOW_PRI = 1;
	
	
	
	public static void makeOrder(ArrayList<Car> cars) throws Exception {
		
		
		TreeMap<Integer, ArrayList<Car>> order = new TreeMap<Integer, ArrayList<Car>>();
		order.put(HIGH_PRI, new ArrayList<Car>());
		order.put(MED_PRI, new ArrayList<Car>());
		order.put(LOW_PRI, new ArrayList<Car>());
	
		
		//setup the order based on which direction a car is going
		for (Car c: cars) {
			
			//turning right
			if (Math.abs(c.getCf() - c.getGt()) == 1) {
				c.setPri(HIGH_PRI);
				order.get(HIGH_PRI).add(c);
			}
			
			//going straight
			else if (Math.abs(c.getCf()) == Math.abs(c.getGt())) {
				c.setPri(MED_PRI);
				order.get(MED_PRI).add(c);
			}
			
			//turning left
			else if ( (Math.abs(c.getCf()) + Math.abs(c.getGt())) == 3) {
				c.setPri(LOW_PRI);
				order.get(LOW_PRI).add(c);
			}
			
			else {
				throw new Exception("Invalid directions given");
			}
			
		}
		
		
		//print out the order of cars from calculation above
		System.out.printf("%4s %12s %20s %20s \n\n", "ID", "Priority", "Arriving From", "Going To" );
			
		for (int pri: order.descendingKeySet()) {
			
			for (Car c: order.get(pri)) {
				System.out.println(c.prettyPrintStatus());
			}
			
			System.out.println();
		}
		
	}
	
	
	
	
	
	public static void main(String[] args) {
		
		/*
		 *  1 -> arriving from North
		 * -1 -> arriving from South
		 *  2 -> arriving from East
		 * -2 -> arriving from West
		 * 
		 */
		
		Car c1 = new Car(1,1,1); //coming north going north
		Car c2 = new Car(1,2,2); //coming north going east
		Car c3 = new Car(-1,-2,3); //coming south going west
		Car c4 = new Car(1,-2,4); //coming north going west
		
		
		for  (int i = 0; i < 10; i++) {
			try {
								
				HashMap<Thread, Car> carThreads = new HashMap<Thread, Car>();
				ArrayList<Car> carsInCurrentEvent = new ArrayList<Car>();
				
				
				Thread t1 = new Thread(c1,"1");
				Thread t2 = new Thread(c2,"2");
				Thread t3 = new Thread(c3,"3");
				Thread t4 = new Thread(c4,"4");

				
				carThreads.put(t1,  c1);
				carThreads.put(t2,  c2);
				carThreads.put(t3,  c3);
				carThreads.put(t4,  c4);
				
				
				//start the threads
				for (Thread t: carThreads.keySet()) {
					t.start();
				}
				
				//set a timeout to only catch a few threads
				for (Thread t: carThreads.keySet()) {
					t.join(200);
					if (!t.isAlive())
						carsInCurrentEvent.add(carThreads.get(t));
					else 
						t.interrupt();
				}
				
				
				System.out.println(" ------------------ # of cars in current Event : " + carsInCurrentEvent.size() + " --------------------------   " );
				
				
				//running the order maker
				makeOrder(carsInCurrentEvent);
			
			}

			catch (Exception e) {
				e.printStackTrace();
			}
		}
			
		}
		
		

	
}




