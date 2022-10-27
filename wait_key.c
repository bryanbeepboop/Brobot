/*
 * wait_key.c
 *
 *  Created on: Mar 18, 2017
 *      Author: steveb
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include "wait_key.h"

/* return true when a key was pressed (or STDIN was closed) and false if the timeout expired */
bool wait_key( int timeout_ms, int * pressed_key )
{
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             select_result;
  fd_set          read_set;
  fd_set          write_set;
  fd_set          error_set;
  struct timeval  timeout;
  bool            return_value;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  FD_ZERO( &read_set );
  FD_ZERO( &write_set );
  FD_ZERO( &error_set );
  FD_SET( STDIN_FILENO, &read_set );
  FD_SET( STDIN_FILENO, &error_set );
  timeout.tv_usec = (timeout_ms * 1000) % 1000000;
  timeout.tv_sec  = (timeout_ms * 1000) / 1000000;
  select_result = select( 1, &read_set, &write_set, &error_set, &timeout );
  switch (select_result)
  {
    case 1:
      return_value = true;
      if (pressed_key > 0)
      {
        if (FD_ISSET( STDIN_FILENO, &error_set ))
        {
          *pressed_key = -1;
        }
        else
        {
        *pressed_key = getchar();
        }
      }
      else
      {
        ; /* not interested in the key value */
      }
      break;

    case 0:
      return_value = false;
      break;

    case -1:
    default:
      return_value = true;
      if (pressed_key > 0)
      {
        *pressed_key = -1;
      }
      else
      {
        ; /* not interested in the key value */
      }
      break;
  }

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return return_value;
}
