package vicSim;

import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.*;
import javax.swing.*;

public class SimulationGUI {
	
	
	private JFrame mainFrame;

	
	
  public SimulationGUI(){
     setupGUI();
  }

   
  private void setupGUI(){
  	
  	 mainFrame = new JFrame("VIC Simulation");
     mainFrame.setSize(600,600);
     mainFrame.setLayout(new GridLayout(5, 5));
     mainFrame.setVisible(true);  
     
     
 }

  
  
  
  
  
  public static void main(String[] args){
  	SimulationGUI swingControlDemo = new SimulationGUI();  
  }
  
}
