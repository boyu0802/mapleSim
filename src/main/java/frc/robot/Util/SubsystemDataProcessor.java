package frc.robot.Util;

import javax.xml.crypto.Data;

public class SubsystemDataProcessor implements Runnable{
    public interface IoRefresher {
        void refreshData();
    }

    public interface  DataReader {
        void readData();        
    }

    public static void createSubsystemDataProcessor(IoRefresher ioRefresher, DataReader dataReader){
        new Thread(new SubsystemDataProcessor(ioRefresher,dataReader)).start();;
    }
    private double looptime = 20.0;
    private double timeStamp = 0.0;
    private IoRefresher ioRefresher;
    private DataReader dataReader;

    public SubsystemDataProcessor(IoRefresher ioRefresher, DataReader dataReader){
        this.ioRefresher = ioRefresher;
        this.dataReader = dataReader;
    }

    public void run() {
        while(true){
            timeStamp = System.currentTimeMillis();
            ioRefresher.refreshData();
            dataReader.readData();

            try {
                var diff = System.currentTimeMillis() - timeStamp;
                if(diff < looptime){
                    Thread.sleep((long) (looptime-diff));
                }
            } catch (InterruptedException e) {}
        }
    }   
}
