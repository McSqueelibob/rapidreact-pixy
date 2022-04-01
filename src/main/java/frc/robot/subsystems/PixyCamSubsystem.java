// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
PORT GUIDE
ports on pixycam
1 is top left closest to the camera
1 2
3 4
5 6
7 8
9 10
1 white, 2 red, 3 red, 4 white, 6 black, 7 black
ports on rio
1 is the ground on the SPI
1 2
3 4
5 6
7 8
9 10
1 black, 2 black, 3 red, 5 white, 6 red, 7 white
*/

public class PixyCamSubsystem extends SubsystemBase {

  private SPI pixy = new SPI(Port.kOnboardCS0);

  /**
   * The words the PixyCam will send.
   */
  private enum pixyWordsTypes
  {
    /**
     * Checksums determine the validity of a set of bytes. When 0, the other words are not set.
     */
    checksum(0),
    /**
     * Signatures are determined by how pixy is tuned. These should be set in Vars.
     * @see https://docs.pixycam.com/wiki/doku.php?id=wiki:v1:signature_tuning_pane 
     * @see frc.robot.Vars
     */
    signature(1),
    /** integers 0-315 representing horizontal screenspace */
    x(2),
    /** integers 0-207 representing vertical screenspace */
    y(3),
    /** integers 0-315 representing horizontal screenspace filled */
    width(4),
    /** integers 0-207 representing vertical screenspace filled */
    height(5);

    private int m_index;
    pixyWordsTypes(int index){m_index = index;}
    /** @return The index of a specific word in the ArrayList */
    public int getIndex(){return m_index;}
  }

  /**
   * ArrayList for storing the bytes the Rio reads off of SPI
   * @see pixyWordsTypes
   */
  private ArrayList<Integer> words = new ArrayList<Integer>();

  /**
   * A counter of the number of errors when reading. Used for debugging.
   */
  private static int checksumError = 0;

  /**
   * Used to track when the data was last read from the SPI bus.
   * If data wants to be read but is stale, the bus should be read again.
   */
  private boolean staleData = true;

  public PixyCamSubsystem()
  {
    pixy.setMSBFirst();
    pixy.setChipSelectActiveLow();
    pixy.setClockRate(1000);
    pixy.setSampleDataOnTrailingEdge();
    pixy.setClockActiveLow();
  }
  
  // The sync byte to get the pixy to talk
  byte PIXY_SYNC_BYTE = 0x5a;

  /**
   * Gets a word from the SPI bus. (The code's origin is unknown.)
   */
  private int getWord()
  {
    int word = 0x00;
    int ret = -1;
    ByteBuffer writeBuf = ByteBuffer.allocateDirect(2);
    writeBuf.order(ByteOrder.BIG_ENDIAN);
    ByteBuffer readBuf = ByteBuffer.allocateDirect(2);
    readBuf.order(ByteOrder.BIG_ENDIAN);
    // String readString = "";
    // String writeString = "";
      
    writeBuf.put(PIXY_SYNC_BYTE);
      
    // Flip the writeBuf so it's ready to be read.
    writeBuf.flip();

    // Send the sync / data bit / 0 to get the Pixy to return data appropriately.
    ret = pixy.transaction(writeBuf, readBuf, 2);
      
    // Set the position back to 0 in the buffer so we read it from the beginning next time.
    readBuf.rewind();
      
    // Store the contents of the buffer in a int that will be returned to the caller.
    word =  (int) (readBuf.getShort() & 0xffff);
      
    // Clear the buffers, not needed, but nice to know they are cleaned out.
    writeBuf.clear();
    readBuf.clear();
    return(word);
  }

  /**
   * Used to the words from the SPI bus that the PixyCam writes.
   * This method updates the words, checksumError, and staleData.
   */
  private void readWords()
  {
    int word;
    int wordsToRead = 0;
    int checksum = 0;
    Boolean syncFound = false;

    // Every iteration of this periodic function will start with a clean ArrayList of words.
    // Then we know the largest object will be at the beginning of this ArrayList.
    words.clear();

    // Read no more than 100 words per periodic function.  100 is just
    // a guess, and the actual amount we can read without interfering 
    // with other periodic robot functions needs to be determined.
    for (int i = 0; i < 100; i++){

      word = getWord();

      // If we have found the start of a frame, read the remaining words of the block.
      // After the last word, break out of the loop to allow other robot functions to run.
      if (wordsToRead > 0){
        if (checksum == 0){
          // The first word after the sync byte is the checksum
          checksum = word;
        }
        else {
          // We subtract from the checksum what the current word is;
          // after all words are read, this checksum will be 0.
          checksum -= word;
        }

        // Add the word to the array list of words
        words.add(word);

        // If we are done reading words of the block, we check the checksum; 
        // if the checksum is bad, dump the array list and count up the checksum error.
        // Either way, we break out of the loop.
        if (--wordsToRead <= 0) {
          if (checksum != 0)
            words.clear();
            checksumError+=1;
          break;
        }
      }
      // 0xaa55 is the sync word
      else if (word == 0xaa55){
        if (syncFound) {
          // We have seen two sync words and we know we have 
          // found the start of a frame of many blocks.  We prepare
          // to read the remaining words on the next pass of the loop.
          wordsToRead = 6;
        }
        else {
          // We found the start of a block, but we need to wait for the
          // start of the frame to make sure we get the largest object.
          syncFound = true;
        }
      }
      else {
        // Clear out the sync flag since we did not read a sync word.
        syncFound = false;
      }

    }

    // If we did not find a valid block, push a null byte to identify the null block.
    if (words.size() == 0) {
      words.add(0);
    }

    staleData = false;
  }

  /**
   * The checksum of the largest block
   * @return checksum
   * @see pixyWordsTypes.checksum
   */
  final private int getChecksumRaw()
  {
    // if the data is stale, the words should be read
    if (staleData) readWords();
    // Why does the checksum call readWords instead of periodic?
    // Calling readWords here ensures that we do not call it more than we need to.
    // Only whenever the pixycam is being used will the SPI bus be read.

    return words.get(pixyWordsTypes.checksum.getIndex());
  }
  
  /**
  * The signature of the largest block
  * @return signature (-1 if checksum is 0)
  * @see pixyWordsTypes.signature
  */
  final private int getSignatureRaw()
  {
    if (getChecksumRaw()==0) return -1;
    return words.get(pixyWordsTypes.signature.getIndex());
 }

  /**
   * The position the largest block in x
   * @return position from 0-315 (-1 if checksum is 0)
   */
  final private int getXRaw()
  {
    if (getChecksumRaw()==0) return -1;
    return words.get(pixyWordsTypes.x.getIndex());
  }
  
  /**
   * The position the largest block in y
   * @return position from 0-207 (-1 if checksum is 0)
   */
  final private int getYRaw()
  {
    if (getChecksumRaw()==0) return -1;
    return words.get(pixyWordsTypes.y.getIndex());
  }
  
  /**
   * The width of the largest block
   * @return width from 0-315 (-1 if checksum is 0)
   */
  final private int getWidthRaw()
  {
    if (getChecksumRaw()==0) return -1;
    // TODO map to angles
    return words.get(pixyWordsTypes.width.getIndex());
  }

  /**
   * The height of the largest block
   * @return height from 0-208 (-1 if checksum is 0)
   */
  final private int getHeightRaw()
  {
    // TODO map to angles
    if (getChecksumRaw()==0) return -1;
    return words.get(pixyWordsTypes.height.getIndex());
  }
  
  /**
   * Returns the distance to ball.
   * @return distance in inches (0 if there is no ball)
   */
  public double getDistance()
  {
    // XXX this requires a definition
    // this value should be calculated
    return 0;
  }

  // /**
  //  * Angle the ball is relative to the camera
  //  * @return angle in degrees (0 if there is no ball)
  //  */
  // public double getAngleX()
  // {
  //   int x = getXRaw();
  //   if (x==-1) return 0;
  //   // maps the raw space to -0.5 to 0.5 and multiplies by the fov
  //   return Constants.PIXY_FOV_HORIZ * (x - 158) / 316;
  // }

  // /**
  //  * Angle the ball is relative to the camera
  //  * @return angle in degrees (0 if there is no ball)
  //  */
  // public double getAngleY()
  // {
  //   int y = getYRaw();
  //   if (y==-1) return 0;
  //   // maps the raw space to -0.5 to 0.5 and multiplies by the fov
  //   return Constants.PIXY_FOV_VERT * (y - 104) / 208;
  // }

  // /**
  //  * Checks if a ball is visible and if it matches our alliance color.
  //  * @return True if the detected ball is our team color.
  //  */
  // public boolean ballVisible()
  // {
  //   switch (DriverStation.getAlliance())
  //   {
  //     case Blue:
  //       return getSignatureRaw() == Vars.PIXY_SIGNATURE_BLUE;
  //     case Red:
  //       return getSignatureRaw() == Vars.PIXY_SIGNATURE_RED;

  //     case Invalid:
  //     default:
  //       return false;
  //   }
  // }

  @Override
  public void periodic()
  {
    readWords();

    SmartDashboard.putNumber("errors", checksumError);
    SmartDashboard.putNumber("signarute", getSignatureRaw());
    SmartDashboard.putNumber("x", getXRaw());
    SmartDashboard.putNumber("y", getYRaw());
    SmartDashboard.putNumber("height", getHeightRaw());
    SmartDashboard.putNumber("width", getWidthRaw());

    staleData = true; // the data is considered stale one loop later
  }

}