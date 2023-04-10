import os
import select

IPC_FIFO_NAME_A = "pipe_a"
IPC_FIFO_NAME_B = "pipe_b"


def get_message(fifo):
    '''Read n bytes from pipe. Note: n=24 is an example'''
    return os.read(fifo, 24)


def process_msg(msg):
    '''Process message read from pipe'''
    return msg


if __name__ == "__main__":
    os.mkfifo(IPC_FIFO_NAME_A)  # Create Pipe A

    try:
        # pipe is opened as read only and in a non-blocking mode
        fifo_a = os.open(IPC_FIFO_NAME_A, os.O_RDONLY | os.O_NONBLOCK)
        print('Pipe A ready')

        while True:
            try:
                fifo_b = os.open(IPC_FIFO_NAME_B, os.O_WRONLY)
                print("Pipe B ready")
                break
            except:
                # Wait until Pipe B has been initialized
                pass

        try:
            poll = select.poll()
            poll.register(fifo_a, select.POLLIN)

            try:
                while True:
                    if (fifo_a, select.POLLIN) in poll.poll(1000):  # Poll every 1 sec
                        # Read from Pipe A
                        msg = get_message(fifo_a)
                        # Process Message
                        msg = process_msg(msg)
                        # Write to Pipe B
                        os.write(fifo_b, msg)

                        print('----- Received from JS -----')
                        print("    " + msg.decode("utf-8"))
            finally:
                poll.unregister(fifo_a)
        finally:
            os.close(fifo_a)
    finally:
        os.remove(IPC_FIFO_NAME_A)
        os.remove(IPC_FIFO_NAME_B)
