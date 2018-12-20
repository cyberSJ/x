package com.cybersj.jws;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class MyTopJPanel extends JPanel implements ActionListener
{
    public static void main(String[] args)
    {   
        javax.swing.SwingUtilities.invokeLater(new Runnable()
        {
            public void run()
            {
                createAndShowGui();
            }
        });
    }
    
    public static void createAndShowGui()
    {
        JFrame topFrame = new JFrame("My JFrame");
        topFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        topFrame.setSize(400, 300);
        
        MyTopJPanel myTopJPanel = new MyTopJPanel();
        topFrame.add(myTopJPanel);
        
        topFrame.setVisible(true);
    }
    
    public MyTopJPanel()
    {
        super(new BorderLayout());
     
        // Add the buttons
        JButton okButton = new JButton("ok");
        okButton.addActionListener(this);
        add(okButton, BorderLayout.SOUTH);
        
        // This works for finding images in a jar or a package.
        ClassLoader classLoader = this.getClass().getClassLoader();
        Icon myIcon = new ImageIcon(
                classLoader.getResource("icon.jpg"));
        
        // This works for finding images in a file system.
//        Icon myIcon = null;
//        try {
//            myIcon = new ImageIcon(
//                ImageIO.read(new File("src/resources/icon.jpg")));
//        } catch (IOException e) {
//            // TODO Auto-generated catch block
//            e.printStackTrace();
//        }
        
        JLabel iconLabel = new JLabel(myIcon);
        
        add(iconLabel, BorderLayout.NORTH);
    }
    
    @Override
    public void actionPerformed(ActionEvent arg0) 
    {
        System.out.println("ok button clicked");
    }

    /**
     * Serial version UID used for the GUI.
     */
    private static final long serialVersionUID = 1L;
}
