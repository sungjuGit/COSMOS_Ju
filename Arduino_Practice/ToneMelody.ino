/*

  Tone Melody

  Plays a melody

  circuit:

  - speaker on digital pin 8

*/

#include "pitches.h"

int buzzPin = 8;

// notes in the melody:
int melody[] = {

  NC4, NG3, NG3, NA3, NG3, 0, NB3, NC4

};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {

  4, 8, 8, 4, 4, 4, 4, 4

};

void setup() {

play_tones();

}

void loop() {

  // no need to repeat the melody.
}


void play_tones() {

    // iterate over the notes of the melody:
    
    for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.

    int noteDuration = 1000 / noteDurations[thisNote];

    tone(buzzPin, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:

    int pauseBetweenNotes = noteDuration * 1.30;

    delay(pauseBetweenNotes);

    // stop the tone playing:

    noTone(buzzPin);

  }
  
}
