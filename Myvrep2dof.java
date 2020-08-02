package pkg2dof;

import coppelia.IntW;
import coppelia.remoteApi;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JTextField;

public class Myvrep2dof {

    double gg = 180;

    remoteApi vrep;

    public void calcFK(double q1, double q2) {
        // double l1 = Double.parseDouble(t.getText());
        double l1 = 10;
        double l2 = 15;
        // double l2 = Double.parseDouble(t2.getText());
        double QQ = q1;
        double QQQ = q2;

        System.out.println("j3=" + QQ);
        System.out.println("j=" + QQQ);
        q1 = (double) (Math.toRadians(QQ));
        q2 = (double) (Math.toRadians(QQQ));
        double x = (double) (l1 * Math.cos(q1));
        x = x;
        x = x + l2 * Math.cos(q1 + q2);
        double y = (double) (l1 * Math.sin(q1));
        y = y;
        y = y + l2 * Math.sin(q1 + q2);
        System.out.println("x=" + x + "y=" + y);

        System.out.println("Forware Kinematics x= " + x + " y= " + y);

    }

    public void start() {

    }

    public void inverse(String dof1, String dof2, double q1, double q2) {
        int clientID;
        try {
            vrep = new remoteApi();
            vrep.simxFinish(-1);

            clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
            if (clientID == -1) {
                System.out.println("Cant connect");
            } else {
                vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
            }
        } catch (Exception ex) {
            System.out.println("Error=" + ex.getMessage());
            return;
        }
        double x = q1;
        double y = q2;
        double l1 = 10;
        double l2 = 15;
        String joint1 = dof1;
        String joint2 = dof2;

        double r = (x * x) + (y * y);
        r = Math.sqrt(r);

        double alpha = Math.atan(y / x);

        double beta = (r * r) + (l1 * l1) - (l2 * l2);
        beta = beta / (2 * r * l1);
        beta = Math.acos(beta);

        double j1 = alpha - beta;
        j1 = Math.toDegrees(j1);

        double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
        j2 = j2 / (2 * l1 * l2);
        j2 = Math.acos(j2);
        j2 = Math.toDegrees(j2);

        IntW handle = new IntW(0);
        //  double code = vrep.simxGetObjectHandle(clientID, "dof1", handle, vrep.simx_opmode_blocking);
        double code = vrep.simxGetObjectHandle(clientID, joint1, handle, vrep.simx_opmode_blocking);
        double d = (double) j1;
        double deg = (double) (d * Math.PI / 180);
        code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

        IntW handle2 = new IntW(0);
        double code2 = vrep.simxGetObjectHandle(clientID, joint2, handle2, vrep.simx_opmode_blocking);
        double d2 = (double) j2;
        double deg2 = (double) (d2 * Math.PI / 180);
        code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

    }

    public void position(String dof1, int q) {
        int clientID;
        try {
            vrep = new remoteApi();
            vrep.simxFinish(-1);

            clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
            if (clientID == -1) {
                System.out.println("Cant connect");
            } else {
                vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
            }
        } catch (Exception ex) {
            System.out.println("Error=" + ex.getMessage());
            return;
        }

        int QQ = q;
        String joint = dof1;

        double d = QQ;
        IntW handle = new IntW(0);
        double code = vrep.simxGetObjectHandle(clientID, joint, handle, vrep.simx_opmode_blocking);
        float deg = (float) (d * Math.PI / 180);
        code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

    }

    public Myvrep2dof() {

        double g = 0;

        JFrame m = new JFrame("2 DOF");
        m.setSize(900, 650);
        m.setLocationRelativeTo(null);
        m.setVisible(true);
        m.setDefaultCloseOperation(m.EXIT_ON_CLOSE);
        m.setLayout(null);
        JLabel jl = new JLabel("Front_Left_Leg");
        jl.setBounds(70, 5, 250, 30);
        JLabel j2 = new JLabel("Front_Right_Leg");
        j2.setBounds(240, 5, 250, 30);
        JLabel j3 = new JLabel("Back_Left_Leg");
        j3.setBounds(460, 5, 250, 30);
        JLabel j4 = new JLabel("Back_Right_Leg");
        j4.setBounds(600, 5, 250, 30);

        //  JTextField t = new JTextField("l");
        //  t.setBackground(Color.white);
        // t.setBounds(100, 30, 60, 30);
        //  JTextField t2 = new JTextField("l2");
        //  t2.setBackground(Color.white);
        //   t2.setBounds(170, 30, 60, 30);
        JTextField t3 = new JTextField("0");
        t3.setBackground(Color.white);
        t3.setBounds(50, 30, 60, 30);

        JTextField t4 = new JTextField("0");
        t4.setBackground(Color.white);
        t4.setBounds(110, 30, 60, 30);

        JTextField t5 = new JTextField("x1");
        t5.setBackground(Color.white);
        t5.setBounds(50, 150, 80, 30);

        JTextField t6 = new JTextField("y1");
        t6.setBackground(Color.white);
        t6.setBounds(140, 150, 80, 30);

        JTextField t7 = new JTextField("value of x1");
        t7.setBackground(Color.white);
        t7.setBounds(50, 200, 80, 30);

        JTextField t8 = new JTextField("value of y1");
        t8.setBackground(Color.white);
        t8.setBounds(140, 200, 80, 30);

        JButton b = new JButton("Forward-K1");
        b.setBounds(50, 80, 100, 30);
        ////////////////////////////////////////////
        JTextField t9 = new JTextField("0");
        t9.setBackground(Color.white);
        t9.setBounds(230, 30, 60, 30);

        JTextField t10 = new JTextField("0");
        t10.setBackground(Color.white);
        t10.setBounds(290, 30, 60, 30);

        JTextField t11 = new JTextField("x2");
        t11.setBackground(Color.white);
        t11.setBounds(240, 150, 60, 30);

        JTextField t12 = new JTextField("y2");
        t12.setBackground(Color.white);
        t12.setBounds(300, 150, 60, 30);

        JTextField t13 = new JTextField("value of x2");
        t13.setBackground(Color.white);
        t13.setBounds(240, 200, 60, 30);

        JTextField t14 = new JTextField("value of y2");
        t14.setBackground(Color.white);
        t14.setBounds(300, 200, 60, 30);

        JTextField t15 = new JTextField("0");
        t15.setBackground(Color.white);
        t15.setBounds(450, 30, 60, 30);

        JTextField t16 = new JTextField("0");
        t16.setBackground(Color.white);
        t16.setBounds(510, 30, 60, 30);

        JTextField t17 = new JTextField("0");
        t17.setBackground(Color.white);
        t17.setBounds(590, 30, 60, 30);

        JTextField t18 = new JTextField("0");
        t18.setBackground(Color.white);
        t18.setBounds(650, 30, 60, 30);

        JTextField t19 = new JTextField("x3");
        t19.setBackground(Color.white);
        t19.setBounds(450, 150, 60, 30);

        JTextField t20 = new JTextField("y3");
        t20.setBackground(Color.white);
        t20.setBounds(520, 150, 60, 30);

        JTextField t21 = new JTextField("x4");
        t21.setBackground(Color.white);
        t21.setBounds(590, 150, 60, 30);

        JTextField t22 = new JTextField("y4");
        t22.setBackground(Color.white);
        t22.setBounds(660, 150, 60, 30);

        JTextField t23 = new JTextField("value of x3");
        t23.setBackground(Color.white);
        t23.setBounds(450, 200, 60, 30);

        JTextField t24 = new JTextField("value ofy3");
        t24.setBackground(Color.white);
        t24.setBounds(520, 200, 60, 30);

        JTextField t25 = new JTextField("value of x4");
        t25.setBackground(Color.white);
        t25.setBounds(590, 200, 60, 30);

        JTextField t26 = new JTextField("value of y4");
        t26.setBackground(Color.white);
        t26.setBounds(660, 200, 60, 30);

        JButton b15 = new JButton("Forward-K2");
        b15.setBounds(240, 80, 100, 30);

        JButton b16 = new JButton("Forward-K3");
        b16.setBounds(450, 80, 100, 30);

        JButton b17 = new JButton("Forward-K4");
        b17.setBounds(590, 80, 100, 30);

        b17.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                // double l1 = Double.parseDouble(t.getText());
                double l1 = 10;
                double l2 = 15;
                // double l2 = Double.parseDouble(t2.getText());
                double QQ = Double.parseDouble(t17.getText());
                double QQQ = Double.parseDouble(t18.getText());
                calcFK(QQ, QQQ);
                System.out.println("j1=" + QQ);
                System.out.println("j2=" + QQQ);
                double q1 = (double) (Math.toRadians(QQ));
                double q2 = (double) (Math.toRadians(QQQ));
                double x = (double) (l1 * Math.cos(q1));
                x = x;
                x = x + l2 * Math.cos(q1 + q2);
                double y = (double) (l1 * Math.sin(q1));
                y = y;
                y = y + l2 * Math.sin(q1 + q2);
                System.out.println("x=" + x + "y=" + y);

                System.out.println("Forware Kinematics x= " + x + " y= " + y);
                String c2 = new Double(Math.round(x * 100.0) / 100.0).toString();
                t21.setText("x= " + c2);
                String c4 = new Double(Math.round(y * 100.0) / 100.0).toString();
                t22.setText("y= " + c4);

                double d = QQ;
                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j7", handle, vrep.simx_opmode_blocking);
                float deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                double d2 = QQQ;
                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j8", handle2, vrep.simx_opmode_blocking);
                float deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);

            }
        });

        b16.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                // double l1 = Double.parseDouble(t.getText());
                double l1 = 10;
                double l2 = 15;
                // double l2 = Double.parseDouble(t2.getText());
                double QQ = Double.parseDouble(t15.getText());
                double QQQ = Double.parseDouble(t16.getText());
                calcFK(QQ, QQQ);
                System.out.println("j1=" + QQ);
                System.out.println("j2=" + QQQ);
                double q1 = (double) (Math.toRadians(QQ));
                double q2 = (double) (Math.toRadians(QQQ));
                double x = (double) (l1 * Math.cos(q1));
                x = x;
                x = x + l2 * Math.cos(q1 + q2);
                double y = (double) (l1 * Math.sin(q1));
                y = y;
                y = y + l2 * Math.sin(q1 + q2);
                System.out.println("x=" + x + "y=" + y);

                System.out.println("Forware Kinematics x= " + x + " y= " + y);
                String c2 = new Double(Math.round(x * 100.0) / 100.0).toString();
                t19.setText("x= " + c2);
                String c4 = new Double(Math.round(y * 100.0) / 100.0).toString();
                t20.setText("y= " + c4);

                double d = QQ;
                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j5", handle, vrep.simx_opmode_blocking);
                float deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                double d2 = QQQ;
                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j6", handle2, vrep.simx_opmode_blocking);
                float deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);
            }
        });

        b15.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent e) {

                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                // double l1 = Double.parseDouble(t.getText());
                double l1 = 10;
                double l2 = 15;
                // double l2 = Double.parseDouble(t2.getText());
                double QQ = Double.parseDouble(t9.getText());
                double QQQ = Double.parseDouble(t10.getText());

                System.out.println("j3=" + QQ);
                System.out.println("j=" + QQQ);
                double q1 = (double) (Math.toRadians(QQ));
                double q2 = (double) (Math.toRadians(QQQ));
                double x = (double) (l1 * Math.cos(q1));
                x = x;
                x = x + l2 * Math.cos(q1 + q2);
                double y = (double) (l1 * Math.sin(q1));
                y = y;
                y = y + l2 * Math.sin(q1 + q2);
                System.out.println("x=" + x + "y=" + y);

                System.out.println("Forware Kinematics x= " + x + " y= " + y);
                String c2 = new Double(Math.round(x * 100.0) / 100.0).toString();
                t11.setText("x= " + c2);
                String c4 = new Double(Math.round(y * 100.0) / 100.0).toString();
                t12.setText("y= " + c4);

                double d = QQ;
                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j3", handle, vrep.simx_opmode_blocking);
                float deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                double d2 = QQQ;
                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j4", handle2, vrep.simx_opmode_blocking);
                float deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);

            }
        });

        ///////////////////////////////////////////
        b.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                // double l1 = Double.parseDouble(t.getText());
                double l1 = 10;
                double l2 = 15;
                // double l2 = Double.parseDouble(t2.getText());
                double QQ = Double.parseDouble(t3.getText());
                double QQQ = Double.parseDouble(t4.getText());
                calcFK(QQ, QQQ);
                System.out.println("j1=" + QQ);
                System.out.println("j2=" + QQQ);
                double q1 = (double) (Math.toRadians(QQ));
                double q2 = (double) (Math.toRadians(QQQ));
                double x = (double) (l1 * Math.cos(q1));
                x = x;
                x = x + l2 * Math.cos(q1 + q2);
                double y = (double) (l1 * Math.sin(q1));
                y = y;
                y = y + l2 * Math.sin(q1 + q2);
                System.out.println("x=" + x + "y=" + y);

                System.out.println("Forware Kinematics x= " + x + " y= " + y);
                String c2 = new Double(Math.round(x * 100.0) / 100.0).toString();
                t5.setText("x= " + c2);
                String c4 = new Double(Math.round(y * 100.0) / 100.0).toString();
                t6.setText("y= " + c4);

                double d = QQ;
                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                float deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                double d2 = QQQ;
                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                float deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);

            }
        });

        JButton b2 = new JButton("Inverse-K1");
        b2.setBounds(50, 280, 100, 30);
        b2.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }
                double x = Double.parseDouble(t7.getText());
                double y = Double.parseDouble(t8.getText());
                double l1 = 10;
                double l2 = 15;

                double r = (x * x) + (y * y);
                r = Math.sqrt(r);
                System.out.println("r=" + r);
                double alpha = Math.atan(y / x);
                System.out.println("alpha=" + alpha);
                double beta = (r * r) + (l1 * l1) - (l2 * l2);
                beta = beta / (2 * r * l1);
                beta = Math.acos(beta);
                System.out.println("beta=" + beta);
                double j1 = alpha - beta;
                j1 = Math.toDegrees(j1);
                System.out.println("j1=" + j1);
                double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                j2 = j2 / (2 * l1 * l2);
                j2 = Math.acos(j2);
                j2 = Math.toDegrees(j2);
                System.out.println("j2=" + j2);

                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                double d = (double) j1;
                double deg = (double) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                double d2 = (double) j2;
                double deg2 = (double) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                System.out.println(deg + " " + deg2);

                String c2 = new Double(Math.round(j1 * 100.0) / 100.0).toString();
                t7.setText("j1= " + c2);

                String c4 = new Double(Math.round(j2 * 100.0) / 100.0).toString();
                t8.setText("j2= " + c4);

            }
        });
        JButton b18 = new JButton("Inverse-K2");
        b18.setBounds(240, 280, 100, 30);
        b18.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {

                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }
                double x = Double.parseDouble(t13.getText());
                double y = Double.parseDouble(t14.getText());
                double l1 = 10;
                double l2 = 15;

                double r = (x * x) + (y * y);
                r = Math.sqrt(r);
                System.out.println("r=" + r);
                double alpha = Math.atan(y / x);
                System.out.println("alpha=" + alpha);
                double beta = (r * r) + (l1 * l1) - (l2 * l2);
                beta = beta / (2 * r * l1);
                beta = Math.acos(beta);
                System.out.println("beta=" + beta);
                double j1 = alpha - beta;
                j1 = Math.toDegrees(j1);
                System.out.println("j3=" + j1);
                double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                j2 = j2 / (2 * l1 * l2);
                j2 = Math.acos(j2);
                j2 = Math.toDegrees(j2);
                System.out.println("j4=" + j2);

                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j3", handle, vrep.simx_opmode_blocking);
                double d = (double) j1;
                double deg = (double) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j4", handle2, vrep.simx_opmode_blocking);
                double d2 = (double) j2;
                double deg2 = (double) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                System.out.println(deg + " " + deg2);

                String c2 = new Double(Math.round(j1 * 100.0) / 100.0).toString();
                t13.setText("j3= " + c2);

                String c4 = new Double(Math.round(j2 * 100.0) / 100.0).toString();
                t14.setText("j4= " + c4);

            }
        });

        JButton b19 = new JButton("Inverse-K3");
        b19.setBounds(450, 280, 100, 30);
        b19.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }
                double x = Double.parseDouble(t23.getText());
                double y = Double.parseDouble(t24.getText());
                double l1 = 10;
                double l2 = 15;

                double r = (x * x) + (y * y);
                r = Math.sqrt(r);
                System.out.println("r=" + r);
                double alpha = Math.atan(y / x);
                System.out.println("alpha=" + alpha);
                double beta = (r * r) + (l1 * l1) - (l2 * l2);
                beta = beta / (2 * r * l1);
                beta = Math.acos(beta);
                System.out.println("beta=" + beta);
                double j1 = alpha - beta;
                j1 = Math.toDegrees(j1);
                System.out.println("j5=" + j1);
                double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                j2 = j2 / (2 * l1 * l2);
                j2 = Math.acos(j2);
                j2 = Math.toDegrees(j2);
                System.out.println("j6=" + j2);

                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j5", handle, vrep.simx_opmode_blocking);
                double d = (double) j1;
                double deg = (double) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j6", handle2, vrep.simx_opmode_blocking);
                double d2 = (double) j2;
                double deg2 = (double) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                System.out.println(deg + " " + deg2);

                String c2 = new Double(Math.round(j1 * 100.0) / 100.0).toString();
                t23.setText("j5= " + c2);

                String c4 = new Double(Math.round(j2 * 100.0) / 100.0).toString();
                t24.setText("j6= " + c4);

            }
        });

        JButton b20 = new JButton("Inverse-K4");
        b20.setBounds(590, 280, 100, 30);

        b20.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {

                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }
                double x = Double.parseDouble(t25.getText());
                double y = Double.parseDouble(t26.getText());
                double l1 = 10;
                double l2 = 15;

                double r = (x * x) + (y * y);
                r = Math.sqrt(r);
                System.out.println("r=" + r);
                double alpha = Math.atan(y / x);
                System.out.println("alpha=" + alpha);
                double beta = (r * r) + (l1 * l1) - (l2 * l2);
                beta = beta / (2 * r * l1);
                beta = Math.acos(beta);
                System.out.println("beta=" + beta);
                double j1 = alpha - beta;
                j1 = Math.toDegrees(j1);
                System.out.println("j7=" + j1);
                double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                j2 = j2 / (2 * l1 * l2);
                j2 = Math.acos(j2);
                j2 = Math.toDegrees(j2);
                System.out.println("j8=" + j2);

                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j7", handle, vrep.simx_opmode_blocking);
                double d = (double) j1;
                double deg = (double) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j8", handle2, vrep.simx_opmode_blocking);
                double d2 = (double) j2;
                double deg2 = (double) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                System.out.println(deg + " " + deg2);

                String c2 = new Double(Math.round(j1 * 100.0) / 100.0).toString();
                t25.setText("j7= " + c2);

                String c4 = new Double(Math.round(j2 * 100.0) / 100.0).toString();
                t26.setText("j8= " + c4);

            }
        });

        JButton b3 = new JButton("Stop simulation");
        b3.setBounds(160, 330, 150, 30);
        b3.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking);
                b2.setText("Inverse-K");

            }

        });

        JTextField t30 = new JTextField("step");
        t30.setBounds(590, 350, 100, 30);

        JButton b21 = new JButton("forward");
        b21.setBounds(590, 400, 100, 30);

        b21.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {

                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }

                ////////////////////////////////////////////////////////////////////////////////////////              
                int a = Integer.parseInt(t30.getText());

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                double l1 = 10;
                double l2 = 15;
                int QQ = 0;
                int QQQ = 0;

                QQ = -30;
                QQQ = 90;
                double q1 = (double) (Math.toRadians(QQ));
                double q2 = (double) (Math.toRadians(QQQ));
                double x = (double) (l1 * Math.cos(q1));
                x = x;
                x = x + l2 * Math.cos(q1 + q2);
                double y = (double) (l1 * Math.sin(q1));
                y = y;
                y = y + l2 * Math.sin(q1 + q2);
                double d = QQ;
                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                float deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);
                double d2 = QQQ;
                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                float deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);

                IntW handle3 = new IntW(0);
                code = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), deg, vrep.simx_opmode_streaming);
                d2 = QQQ;
                IntW handle4 = new IntW(0);
                code2 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), deg2, vrep.simx_opmode_streaming);

                IntW handle5 = new IntW(0);
                code = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), deg, vrep.simx_opmode_streaming);
                d2 = QQQ;
                IntW handle6 = new IntW(0);
                code2 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), deg2, vrep.simx_opmode_streaming);

                IntW handle7 = new IntW(0);
                code = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                deg = (float) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), deg, vrep.simx_opmode_streaming);
                d2 = QQQ;
                IntW handle8 = new IntW(0);
                code2 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                deg2 = (float) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), deg2, vrep.simx_opmode_streaming);
                delay(1000);

                int n = 1;
                for (int i = 1; i <= a; i++) {

                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////              
                    QQ = -60;
                    QQQ = 120;

                    d = QQ;
                    d2 = QQQ;

                    code2 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    code = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), deg, vrep.simx_opmode_streaming);

                    d2 = 90;
                    code2 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    //////////////////////////////////////////////////////////////////////////////////////////////////
                    QQ = -60;
                    QQQ = 120;

                    d = QQ;
                    d2 = QQQ;

                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                    d2 = 90;
                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    /////////////////////////////////////////////////////////////////////////////
                    QQ = -60;
                    QQQ = 120;

                    d = QQ;
                    d2 = QQQ;

                    code2 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    code = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), deg, vrep.simx_opmode_streaming);

                    d2 = 90;
                    code2 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), deg2, vrep.simx_opmode_streaming);

                    delay(50);

                    /////////////////////////////////////////////////////////////////////////////////
                    QQ = -60;
                    QQQ = 120;

                    d = QQ;
                    d2 = QQQ;

                    code2 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    code = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), deg, vrep.simx_opmode_streaming);

                    d2 = 90;
                    code2 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), deg2, vrep.simx_opmode_streaming);
                    ////////////////////////////////////////

                    ///////////////////////////////////////////////////////////////////////
                    ///////////////////////////////////////////////////////////////////////
                    QQ = -30;
                    QQQ = 90;

                    delay(0000);

                    d = QQ;

                    code = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), deg, vrep.simx_opmode_streaming);
                    d2 = QQQ;

                    code2 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), deg2, vrep.simx_opmode_streaming);

                    code = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), deg2, vrep.simx_opmode_streaming);

                    code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), deg2, vrep.simx_opmode_streaming);

                    code = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                    deg = (float) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    deg2 = (float) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), deg2, vrep.simx_opmode_streaming);

                    delay(50);

                    t30.setText("" + n);
                    n++;
                }

                //////////////////////////////////////////////////////////////////////////////////////////               
            }
        });

        JButton b22 = new JButton("backward");
        b22.setBounds(590, 450, 100, 30);
        b22.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = Integer.parseInt(t30.getText());
                /*   inverse("j1", "j2", 17.99, -1.16);
                inverse("j3", "j4", 17.99, -1.16);
                inverse("j5", "j6", 17.99, -1.16);
                inverse("j7", "j8", 17.99, -1.16); */
                int clientID;
                try {
                    vrep = new remoteApi();
                    vrep.simxFinish(-1);

                    clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 5000, 5);
                    if (clientID == -1) {
                        System.out.println("Cant connect");
                    } else {
                        vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
                    }
                } catch (Exception ex) {
                    System.out.println("Error=" + ex.getMessage());
                    return;
                }
                double x = 17.99;
                double y = -1.16;
                double l1 = 10;
                double l2 = 15;

                double r = (x * x) + (y * y);
                r = Math.sqrt(r);
                System.out.println("r=" + r);
                double alpha = Math.atan(y / x);
                System.out.println("alpha=" + alpha);
                double beta = (r * r) + (l1 * l1) - (l2 * l2);
                beta = beta / (2 * r * l1);
                beta = Math.acos(beta);
                System.out.println("beta=" + beta);
                double j1 = alpha - beta;
                j1 = Math.toDegrees(j1);
                System.out.println("j1=" + j1);
                double j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                j2 = j2 / (2 * l1 * l2);
                j2 = Math.acos(j2);
                j2 = Math.toDegrees(j2);
                System.out.println("j2=" + j2);

                IntW handle = new IntW(0);
                double code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                double d = (double) j1;
                double deg = (double) (d * Math.PI / 180);
                code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle2 = new IntW(0);
                double code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                double d2 = (double) j2;
                double deg2 = (double) (d2 * Math.PI / 180);
                code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                IntW handle3 = new IntW(0);
                double code3 = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                code3 = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle4 = new IntW(0);
                double code4 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                code4 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                IntW handle5 = new IntW(0);
                double code5 = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                code5 = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle6 = new IntW(0);
                double code6 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                code6 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                IntW handle7 = new IntW(0);
                double code7 = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                code7 = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), (float) deg, vrep.simx_opmode_streaming);

                IntW handle8 = new IntW(0);
                double code8 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                code8 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                delay(1000);
                int n = 1;
                for (int i = 1; i <= a; i++) {

                    /*  inverse("j1", "j2", 12.5, 4.33);
                    delay(50);
                    inverse("j1", "j2", 16.16, 7.99);
                    delay(50);

                    inverse("j5", "j6", 12.5, 4.33);
                    delay(50);
                    inverse("j5", "j6", 16.16, 7.99);
                    delay(50);

                    inverse("j3", "j4", 12.5, 4.33);
                    delay(50);
                    inverse("j3", "j4", 16.16, 7.99);
                    delay(50);

                    inverse("j7", "j8", 12.5, 4.33);
                    delay(50);
                    inverse("j7", "j8", 16.16, 7.99);
                    delay(50);

                    inverse("j1", "j2", 17.99, -1.16);
                    inverse("j3", "j4", 17.99, -1.16);
                    inverse("j5", "j6", 17.99, -1.16);
                    inverse("j7", "j8", 17.99, -1.16); */
                    x = 12.5;
                    y = 4.33;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);
                    System.out.println("r=" + r);
                    alpha = Math.atan(y / x);
                    System.out.println("alpha=" + alpha);
                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);
                    System.out.println("beta=" + beta);
                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);
                    System.out.println("j1=" + j1);
                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);
                    System.out.println("j2=" + j2);

                    code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);
/////////////////////////////////////////////////////////////////
                    x = 16.16;
                    y = 7.99;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);
                    System.out.println("r=" + r);
                    alpha = Math.atan(y / x);
                    System.out.println("alpha=" + alpha);
                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);
                    System.out.println("beta=" + beta);
                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);
                    System.out.println("j1=" + j1);
                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);
                    System.out.println("j2=" + j2);

                    code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    ////////////////////////////////////////
                    x = 12.5;
                    y = 4.33;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code5 = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code5 = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code6 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code6 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    /////////////////////////////////////
                    x = 16.16;
                    y = 7.99;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code5 = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code5 = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code6 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code6 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    ///////////////////////////////
                    x = 12.5;
                    y = 4.33;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code3 = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code3 = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code4 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code4 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    ///////////////////////
                    x = 16.16;
                    y = 7.99;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code3 = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code3 = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code4 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code4 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    ///////
                    x = 12.5;
                    y = 4.33;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code7 = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code7 = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code8 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code8 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);
                    ////////////

                    x = 16.16;
                    y = 7.99;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);

                    code7 = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code7 = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code8 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code8 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), (float) deg2, vrep.simx_opmode_streaming);
                    delay(50);

                    //////////////////////////////////////////////////////
                    x = 17.99;
                    y = -1.16;
                    l1 = 10;
                    l2 = 15;

                    r = (x * x) + (y * y);
                    r = Math.sqrt(r);

                    alpha = Math.atan(y / x);

                    beta = (r * r) + (l1 * l1) - (l2 * l2);
                    beta = beta / (2 * r * l1);
                    beta = Math.acos(beta);

                    j1 = alpha - beta;
                    j1 = Math.toDegrees(j1);

                    j2 = (x * x) + (y * y) - (l1 * l1) - (l2 * l2);
                    j2 = j2 / (2 * l1 * l2);
                    j2 = Math.acos(j2);
                    j2 = Math.toDegrees(j2);
                    System.out.println("j2=" + j2);

                    code = vrep.simxGetObjectHandle(clientID, "j1", handle, vrep.simx_opmode_blocking);
                    d = (double) j1;
                    deg = (double) (d * Math.PI / 180);
                    code = vrep.simxSetJointTargetPosition(clientID, handle.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code2 = vrep.simxGetObjectHandle(clientID, "j2", handle2, vrep.simx_opmode_blocking);
                    d2 = (double) j2;
                    deg2 = (double) (d2 * Math.PI / 180);
                    code2 = vrep.simxSetJointTargetPosition(clientID, handle2.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                    code3 = vrep.simxGetObjectHandle(clientID, "j3", handle3, vrep.simx_opmode_blocking);
                    code3 = vrep.simxSetJointTargetPosition(clientID, handle3.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code4 = vrep.simxGetObjectHandle(clientID, "j4", handle4, vrep.simx_opmode_blocking);
                    code4 = vrep.simxSetJointTargetPosition(clientID, handle4.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                    code5 = vrep.simxGetObjectHandle(clientID, "j5", handle5, vrep.simx_opmode_blocking);
                    code5 = vrep.simxSetJointTargetPosition(clientID, handle5.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code6 = vrep.simxGetObjectHandle(clientID, "j6", handle6, vrep.simx_opmode_blocking);
                    code6 = vrep.simxSetJointTargetPosition(clientID, handle6.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                    code7 = vrep.simxGetObjectHandle(clientID, "j7", handle7, vrep.simx_opmode_blocking);
                    code7 = vrep.simxSetJointTargetPosition(clientID, handle7.getValue(), (float) deg, vrep.simx_opmode_streaming);

                    code8 = vrep.simxGetObjectHandle(clientID, "j8", handle8, vrep.simx_opmode_blocking);
                    code8 = vrep.simxSetJointTargetPosition(clientID, handle8.getValue(), (float) deg2, vrep.simx_opmode_streaming);

                    t30.setText("" + a);
                    n++;
                }
            }
        });

        JTextField t31 = new JTextField("0");
        t31.setBounds(400, 350, 100, 30);

        JButton b23 = new JButton("Right");
        b23.setBounds(455, 400, 100, 30);

        b23.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = Integer.parseInt(t31.getText());

                position("j9", -a);

            }
        });

        JButton b24 = new JButton("Left");
        b24.setBounds(350, 400, 100, 30);

        b24.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = Integer.parseInt(t31.getText());

                position("j9", a);
            }
        });

        JButton b25 = new JButton("Up/Down");
        b25.setBounds(400, 450, 100, 30);

        b25.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = Integer.parseInt(t31.getText());

                if (a >= 90 && a <= 180) {

                    position("j10", a);
                }

            }
        });

        JButton b26 = new JButton("Grab");
        b26.setBounds(400, 500, 100, 30);

        b26.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = 155;

                position("j12", a);

            }
        });

        JButton b27 = new JButton("Release");
        b27.setBounds(400, 550, 100, 30);

        b27.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int a = 180;

                position("j12", a);

            }
        });

        // m.add(t);
        // m.add(t2);
        m.add(t3);
        m.add(t4);
        m.add(t7);
        m.add(t8);
        m.add(b);
        m.add(t5);
        m.add(t6);
        m.add(b2);
        m.add(b3);
        /////////  
        m.add(jl);
        m.add(j2);
        m.add(j3);
        m.add(j4);
        ///// 
        m.add(t9);
        m.add(t10);
        m.add(t11);
        m.add(t12);
        m.add(t13);
        m.add(t14);
        m.add(t15);
        m.add(t16);
        m.add(t17);
        m.add(t18);
        m.add(t19);
        m.add(t20);
        m.add(t21);
        m.add(t22);
        m.add(t23);
        m.add(t24);
        m.add(t25);
        m.add(t26);
        m.add(t30);
        m.add(t31);

        m.add(b15);
        m.add(b16);
        m.add(b17);
        m.add(b18);
        m.add(b19);
        m.add(b20);
        m.add(b21);
        m.add(b22);
        m.add(b23);
        m.add(b24);
        m.add(b25);
        m.add(b26);
        m.add(b27);

        m.repaint();
        m.validate();

    }

    private void delay(int ms) {
        try {
            Thread.sleep(ms);
        } catch (Exception ex) {

        }
    }

    public static void main(String[] args) {

        Myvrep2dof m = new Myvrep2dof();
        //  m.delay(30);

    }
}
