package frc.robot.hardware;

import edu.wpi.first.wpilibj.AnalogInput;

public class IntakeSensor {
    private AnalogInput m_analog1, m_analog2, m_analog3, m_analog4;

        public IntakeSensor() {
            m_analog1 = new AnalogInput(0);
            m_analog2 = new AnalogInput(1);
            m_analog3 = new AnalogInput(2);
            m_analog4 = new AnalogInput(3);
        }

        /**
         * @return the average values of all the sensors in an integer array
         */
        public int[] getSensorValue(){
            int[] AverageValues = {m_analog1.getAverageValue(), m_analog2.getAverageValue(), m_analog3.getAverageValue(), m_analog4.getAverageValue()};
            return AverageValues;
        }

        /**
         * @return whether any of the four analog inputs detects something in the intake
         */
        public boolean isDetected(){ 
            int[] AverageValues = getSensorValue();

            for (int analogValue : AverageValues){
                if (analogValue < 2500) { return true; }
            }
            return false;
        }
}
