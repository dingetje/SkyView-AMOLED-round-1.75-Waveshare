## Steps to Update the Buddy List

1. **Connect to SkyView WiFi**
    - Power on your SkyView device.
    - Connect your computer or phone to the SkyView WiFi access point (available for 10 minutes after powering on).

2. **Access the Web Interface**
    - Open your web browser.
    - Navigate to [http://192.168.1.1](http://192.168.1.1).

 ![alt text](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/tree/main/Documentation/SkyViewFrontPage.png "SkyView Front Page")    

3. **Open the Buddy List Page**
    - Click on the **Buddy List** link.
 ![alt text](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/tree/main/Documentation/images/upload_page.png "Upload Page")

4. **Download the Existing Buddy List (Optional)**
    - If you have previously uploaded a buddy list, you can download it.
    - Click **Download**.
    - Either right-click and select **Save As...** or copy the text into a Notepad file.

    ![alt text](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/tree/main/Documentation/images/buddy_list_saveas.png "Buddy List Save As")

5. **Edit the Buddy List**
    - Open the file in a text editor (e.g., Notepad).
    - Each line should have the format:  
      ```
      HEX_ID,Name
      ```
      *Example:* `1A2B3C,John Doe`
    - **Note:** No spaces after the comma. But space between Firsname and lastname.
    - **Note:** You can also download example list from here.

6. **Upload the Updated Buddy List**
    - Click **Choose File** and select your edited file.
    - Click **Upload**.

7. **Confirmation**
    - You will see a "File Uploaded Successfully!" message.
    - Changes take effect immediately—no restart required.

---

**Tips:**
- Double-check the format before uploading.
- If you encounter issues, refresh the page and try again.
