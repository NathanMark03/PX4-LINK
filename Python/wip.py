class Message(BaseModel):
    """
    Docstring for data
    """
    data_string: list[float]

class Translator:
    """
    Docstring for Translator

    Utility class
    """
    @staticmethod
    def write(send: HIL_SEND) -> Message:
        """
        Docstring for write
        
        :param TODO
        """
        msg = Message()

        # create logic to translate mavlink messages to string #

        return msg

    @staticmethod
    def read(msg: Message) -> HIL_REC:
        """
        Docstring for read
        
        :param TODO
        """
        rec = HIL_REC()

        # create logic to translate string to mavlink message #

        return rec